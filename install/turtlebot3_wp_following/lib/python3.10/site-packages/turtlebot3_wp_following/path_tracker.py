#!/usr/bin/env python3
"""Nav2-free pure-pursuit tracker for a cached path.

This node reads the waypoints YAML file produced by :mod:`path_cacher` (or any
file in the ``wp_following.yaml`` format) and drives the robot along the stored
path using a classic **pure pursuit** controller - no Nav2, no global planner,
no map required.

Pipeline
--------
1. Load the path from ``path_file`` (default ``config/cached_path.yaml``).
2. Look up the robot pose in the ``map`` frame via TF (``map`` -> ``base_link``
   by default; override with ``robot_frame`` / ``map_frame``).
3. Pure pursuit:
   - find the point on the path that is ``lookahead`` metres ahead of the robot,
   - compute the curvature command ``w = 2 * sin(alpha) / L``,
   - saturate linear / angular velocity to the configured limits,
   - slow down for sharp turns and near the goal.
4. Stop when the goal is reached (within ``goal_tolerance``) and, optionally,
   loop.

The robot is driven by publishing ``geometry_msgs/Twist`` on ``cmd_vel``
(``cmd_vel_topic``).  Nothing here depends on Nav2 being running.

Usage::

    ros2 run turtlebot3_wp_following path_tracker \
        --ros-args -p path_file:=/abs/path/cached_path.yaml
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q):
    """Planar yaw (rad) from a quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _number(value):
    return float(value) if value is not None else 0.0


class PathTracker(Node):
    """Drive the robot along a cached path with pure pursuit."""

    def __init__(self):
        super().__init__('path_tracker')
        self.get_logger().info('=== pure-pursuit path_tracker loaded ===')

        # ------------------------------------------------------------------
        # Defaults
        # ------------------------------------------------------------------
        try:
            share = get_package_share_directory('turtlebot3_wp_following')
            default_file = os.path.join(share, 'config', 'cached_path.yaml')
        except Exception:
            default_file = os.path.expanduser('~/cached_path.yaml')
        # Prefer the user's home copy if present (matches the launch helper).
        user_file = os.path.expanduser('~/cached_path.yaml')
        if os.path.isfile(user_file):
            default_file = user_file

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('path_file', default_file)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        # Forward distance used to pick the target point on the path.
        self.declare_parameter('lookahead', 0.35)
        # Control loop rate (Hz).
        self.declare_parameter('control_rate', 20.0)
        # Velocity limits.
        self.declare_parameter('max_linear', 0.22)
        self.declare_parameter('min_linear', 0.05)
        self.declare_parameter('max_angular', 1.5)
        # Slow down when the heading error is larger than this (rad).
        self.declare_parameter('slowdown_angle', 0.6)
        # Distance (m) within which the goal is considered reached.
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('loop', False)
        # If no TF is available yet, wait instead of spamming errors.
        self.declare_parameter('startup_delay', 3.0)

        self._path_file = self.get_parameter(
            'path_file').get_parameter_value().string_value
        self._map_frame = self.get_parameter(
            'map_frame').get_parameter_value().string_value
        self._robot_frame = self.get_parameter(
            'robot_frame').get_parameter_value().string_value
        self._lookahead = self.get_parameter(
            'lookahead').get_parameter_value().double_value
        self._control_rate = self.get_parameter(
            'control_rate').get_parameter_value().double_value
        self._max_linear = self.get_parameter(
            'max_linear').get_parameter_value().double_value
        self._min_linear = self.get_parameter(
            'min_linear').get_parameter_value().double_value
        self._max_angular = self.get_parameter(
            'max_angular').get_parameter_value().double_value
        self._slowdown_angle = self.get_parameter(
            'slowdown_angle').get_parameter_value().double_value
        self._goal_tolerance = self.get_parameter(
            'goal_tolerance').get_parameter_value().double_value
        self._loop = self.get_parameter('loop').get_parameter_value().bool_value
        self._startup_delay = self.get_parameter(
            'startup_delay').get_parameter_value().double_value

        # ------------------------------------------------------------------
        # Path
        # ------------------------------------------------------------------
        self._path = self._load_path(self._path_file)
        if len(self._path) < 2:
            self.get_logger().error(
                f'Path from "{self._path_file}" has < 2 points '
                f'({len(self._path)}). Nothing to do.')
        else:
            self.get_logger().info(
                f'Loaded {len(self._path)} path points from '
                f'"{self._path_file}".')

        self._index = 0            # nearest-path-point cursor
        self._finished = False

        # ------------------------------------------------------------------
        # Publishers / TF
        # ------------------------------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        rate = self._control_rate if self._control_rate > 0.0 else 20.0
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self._tf_ready = False

    # ----------------------------------------------------------------------
    # Loading
    # ----------------------------------------------------------------------
    def _load_path(self, path):
        """Load a waypoints YAML file into a list of ``(x, y)`` tuples.

        Reads the same format produced by :mod:`path_cacher` and consumed by
        :mod:`waypoint_follower` (``waypoints:`` -> ``pose`` /
        ``orientation``).  Also tolerates a plain list of ``[x, y, yaw]``.
        """
        if not os.path.isfile(path):
            self.get_logger().warn(f'Path file not found: {path}')
            return []

        with open(path, 'r') as handle:
            data = yaml.safe_load(handle) or {}

        raw = data.get('waypoints', data) if isinstance(data, dict) else data
        entries = []

        if isinstance(raw, dict):
            def _key(name):
                digits = ''.join(c for c in str(name) if c.isdigit())
                return int(digits) if digits else 0
            entries = [raw[k] for k in sorted(raw, key=_key)]
        elif isinstance(raw, list):
            entries = raw
        else:
            self.get_logger().error(
                'Path file must contain a "waypoints" mapping or list.')
            return []

        points = []
        for entry in entries:
            point = self._parse_entry(entry)
            if point is not None:
                points.append(point)
        return points

    def _parse_entry(self, entry):
        if isinstance(entry, dict) and 'pose' in entry:
            pose = entry['pose']
            if isinstance(pose, (list, tuple)) and len(pose) >= 2:
                return (_number(pose[0]), _number(pose[1]))
        if isinstance(entry, dict) and 'x' in entry and 'y' in entry:
            return (_number(entry.get('x')), _number(entry.get('y')))
        if isinstance(entry, (list, tuple)) and len(entry) >= 2:
            return (_number(entry[0]), _number(entry[1]))
        return None

    # ----------------------------------------------------------------------
    # TF helpers
    # ----------------------------------------------------------------------
    def _robot_pose(self):
        """Return ``(x, y, yaw)`` of the robot in the map frame, or ``None``."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        yaw = yaw_from_quaternion(tf.transform.rotation)
        return (t.x, t.y, yaw)

    # ----------------------------------------------------------------------
    # Pure pursuit
    # ----------------------------------------------------------------------
    def _nearest_index(self, x, y, start):
        """Index of the path point nearest to ``(x, y)``.

        Searches forward from ``start`` so the cursor only ever advances,
        preventing the robot from being pulled back to an earlier segment at
        self-intersections.
        """
        best = start
        best_d = float('inf')
        for i in range(start, len(self._path)):
            px, py = self._path[i]
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d = d
                best = i
        return best

    def _lookahead_point(self, x, y):
        """First path point at least ``lookahead`` metres from ``(x, y)``.

        Walks forward through the path (with linear interpolation inside the
        segment where the lookahead circle is crossed) and returns the target
        ``(tx, ty, index)``.
        """
        start = self._nearest_index(x, y, self._index)
        self._index = start

        for i in range(start, len(self._path) - 1):
            ax, ay = self._path[i]
            bx, by = self._path[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            if seg < 1e-9:
                continue
            # Project the robot onto the (infinite) line of this segment.
            ux = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / seg
            # Distance from robot to the closest point on the segment.
            proj_x = ax + (max(0.0, min(seg, ux)) / seg) * (bx - ax)
            proj_y = ay + (max(0.0, min(seg, ux)) / seg) * (by - ay)
            if math.hypot(proj_x - x, proj_y - y) <= self._lookahead:
                continue
            # This segment crosses the lookahead circle - interpolate the point.
            # Solve |a + s*(b-a) - p| = lookahead for s in [0, 1].
            dx = bx - ax
            dy = by - ay
            fx = ax - x
            fy = ay - y
            a = dx * dx + dy * dy
            b = 2.0 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - self._lookahead ** 2
            disc = b * b - 4.0 * a * c
            if disc < 0.0:
                continue
            s = (-b + math.sqrt(disc)) / (2.0 * a)
            if 0.0 <= s <= 1.0:
                return (ax + s * dx, ay + s * dy, i)

        # No crossing found: aim at the final point.
        gx, gy = self._path[-1]
        return (gx, gy, len(self._path) - 1)

    def _compute_command(self, x, y, yaw):
        """Return a ``Twist`` for the current robot pose (may be zero)."""
        tx, ty, index = self._lookahead_point(x, y)
        self._index = index

        goal_x, goal_y = self._path[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)

        cmd = Twist()

        if goal_dist <= self._goal_tolerance:
            return cmd, True  # reached

        dx = tx - x
        dy = ty - y
        target_dist = math.hypot(dx, dy)
        if target_dist < 1e-6:
            return cmd, False

        # Heading error between robot and the lookahead target.
        target_yaw = math.atan2(dy, dx)
        alpha = math.atan2(math.sin(target_yaw - yaw),
                           math.cos(target_yaw - yaw))

        # Pure-pursuit curvature -> angular velocity.
        curvature = 2.0 * math.sin(alpha) / max(self._lookahead, 1e-3)
        linear = self._max_linear
        angular = curvature * linear

        # Slow down for sharp turns (proportional to how far off-heading).
        if abs(alpha) > self._slowdown_angle:
            linear = self._min_linear
        else:
            scale = 1.0 - (1.0 - self._min_linear / self._max_linear) * \
                (abs(alpha) / self._slowdown_angle)
            linear = self._max_linear * max(scale, self._min_linear /
                                            self._max_linear)

        # Ease off near the goal.
        if goal_dist < 3.0 * self._goal_tolerance:
            linear = min(linear, self._min_linear)

        # Very large heading errors: rotate in place first.
        if abs(alpha) > math.pi / 2.0:
            linear = 0.0

        angular = max(-self._max_angular, min(self._max_angular, angular))

        cmd.linear.x = max(0.0, min(self._max_linear, linear))
        cmd.angular.z = angular
        return cmd, False

    # ----------------------------------------------------------------------
    # Timer
    # ----------------------------------------------------------------------
    def _on_timer(self):
        if self._finished or len(self._path) < 2:
            self._publish_stop()
            return

        pose = self._robot_pose()
        if pose is None:
            if not self._tf_ready:
                self.get_logger().info(
                    f'Waiting for TF {self._map_frame} -> '
                    f'{self._robot_frame} ...', once=True)
            return
        self._tf_ready = True

        x, y, yaw = pose
        cmd, reached = self._compute_command(x, y, yaw)

        if reached:
            self._publish_stop()
            if self._loop:
                self.get_logger().info('Goal reached - looping to start.')
                self._index = 0
            else:
                self.get_logger().info('Goal reached - stopping.')
                self._finished = True
            return

        self._cmd_pub.publish(cmd)

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure the robot stops when the node exits.
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
