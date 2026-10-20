#!/usr/bin/env python3
"""Cache a Nav2 global path published on ``/plan`` into a waypoints YAML file.

Workflow
--------
1. Bring up Gazebo + Nav2 + the :mod:`waypoint_follower` as usual.
2. Run this node at the same time::

       ros2 run turtlebot3_wp_following path_cacher

   (or ``ros2 launch turtlebot3_wp_following cache_path.launch.py``)

3. Drive *one* waypoint-follower run.  While Nav2 plans the route it publishes
   the global path (``nav_msgs/Path``) on ``/plan``.  Every message is
   down-sampled and written to a YAML file in the *same format* used by
   ``wp_following.yaml`` (``waypoints:`` / ``pose`` / ``orientation``), so the
   cached file can be fed straight back into the follower **or** tracked by the
   Nav2-free :mod:`path_tracker`.

Only the ``/plan`` path is used - no Nav2 dependency is required at runtime
beyond publishing that topic.  By default the newest path overwrites the file,
so the last plan of the run wins.

The output file lives in the package *source* directory by default (falls back
to ``~/cached_path.yaml`` when the source tree cannot be found), so it survives
``colcon build`` invocations.
"""

import math
import os

import rclpy
import yaml
from nav_msgs.msg import Path
from rclpy.node import Node


def quaternion_to_yaw(q):
    """Return the planar yaw (rad) of a quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    """Convert a planar yaw (rad) into a quaternion ``[w, x, y, z]`` list."""
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


class PathCacher(Node):
    """Down-sample ``/plan`` and save it as a waypoints YAML file."""

    def __init__(self):
        super().__init__('path_cacher')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        default_out = self._default_output_file()
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('output_file', default_out)
        # Minimum spacing (metres) between consecutive cached waypoints.  This
        # is the "down-sample" knob: a dense Nav2 path (hundreds of points) is
        # reduced to a handful of evenly-spaced poses.
        self.declare_parameter('min_spacing', 0.20)
        # Always keep the very first and last pose of the path, even if they
        # are closer than ``min_spacing`` to their neighbour.
        self.declare_parameter('keep_endpoints', True)
        # Only (re)write the file when the incoming path differs from the
        # previously cached one by more than this many poses.  Avoids hammering
        # the disk while Nav2 keeps republishing the same plan.  Set to 0 to
        # write on every message.
        self.declare_parameter('min_pose_change', 1)

        self._plan_topic = self.get_parameter(
            'plan_topic').get_parameter_value().string_value
        self._output_file = self.get_parameter(
            'output_file').get_parameter_value().string_value
        self._min_spacing = self.get_parameter(
            'min_spacing').get_parameter_value().double_value
        self._keep_endpoints = self.get_parameter(
            'keep_endpoints').get_parameter_value().bool_value
        self._min_pose_change = self.get_parameter(
            'min_pose_change').get_parameter_value().integer_value

        self._last_count = -1

        # ------------------------------------------------------------------
        # Subscription
        # ------------------------------------------------------------------
        self.create_subscription(Path, self._plan_topic, self._plan_callback, 10)

        self.get_logger().info(
            f'path_cacher listening on "{self._plan_topic}". '
            f'Cached waypoints will be written to:\n    {self._output_file}')

    # ----------------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------------
    @staticmethod
    def _default_output_file():
        """Prefer the package source dir, else fall back to ``~/``.

        The source directory is derived from ``__file__``::

            <ws>/src/turtlebot3/turtlebot3_wp_following/
                turtlebot3_wp_following/path_cacher.py
        """
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_dir = os.path.join(pkg_dir, 'config')
        if os.path.isdir(config_dir):
            return os.path.join(config_dir, 'cached_path.yaml')
        return os.path.expanduser('~/cached_path.yaml')

    def _downsample(self, poses):
        """Return a list of ``PoseStamped`` reduced to ``min_spacing`` spacing.

        ``poses`` is the full list from ``nav_msgs/Path``.  The first pose is
        always kept; afterwards a pose is kept only once the accumulated
        distance from the last kept pose reaches ``min_spacing``.  The final
        pose is appended when ``keep_endpoints`` is true.
        """
        if not poses:
            return []

        keep = [poses[0]]
        last_kept = poses[0]

        for pose in poses[1:]:
            dx = pose.pose.position.x - last_kept.pose.position.x
            dy = pose.pose.position.y - last_kept.pose.position.y
            if math.hypot(dx, dy) >= self._min_spacing:
                keep.append(pose)
                last_kept = pose

        if self._keep_endpoints and len(keep) > 0:
            if keep[-1] is not poses[-1]:
                keep.append(poses[-1])
        return keep

    def _write_file(self, poses, frame_id):
        """Write the down-sampled poses as a ``waypoints:`` YAML file."""
        waypoints = {}
        for index, pose in enumerate(poses):
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            q = pose.pose.orientation
            # Stored as [w, x, y, z] to match wp_following.yaml exactly.
            orientation = [
                round(float(q.w), 6),
                round(float(q.x), 6),
                round(float(q.y), 6),
                round(float(q.z), 6),
            ]
            waypoints[f'waypoint{index}'] = {
                'pose': [round(float(x), 6), round(float(y), 6), round(float(z), 6)],
                'orientation': orientation,
            }

        document = {
            'frame_id': frame_id,
            'source': self._plan_topic,
            'min_spacing': self._min_spacing,
            'waypoints': waypoints,
        }

        os.makedirs(os.path.dirname(self._output_file), exist_ok=True)
        with open(self._output_file, 'w') as handle:
            yaml.safe_dump(document, handle, default_flow_style=False,
                           sort_keys=False)

    # ----------------------------------------------------------------------
    # Callback
    # ----------------------------------------------------------------------
    def _plan_callback(self, msg):
        if not msg.poses:
            return

        # Skip tiny/duplicate plans (Nav2 often republishes the same path).
        if (self._min_pose_change > 0
                and self._last_count >= 0
                and abs(len(msg.poses) - self._last_count)
                < self._min_pose_change):
            # Same size as last time - assume unchanged and skip the write.
            return

        reduced = self._downsample(msg.poses)
        self._write_file(reduced, msg.header.frame_id or 'map')
        self._last_count = len(msg.poses)

        self.get_logger().info(
            f'Cached path: {len(msg.poses)} poses -> {len(reduced)} waypoints '
            f'(frame "{msg.header.frame_id}") -> {self._output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = PathCacher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
