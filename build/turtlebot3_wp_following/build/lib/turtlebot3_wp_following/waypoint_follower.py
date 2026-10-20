#!/usr/bin/env python3
"""Waypoint follower for TurtleBot3 built on top of Nav2.

The node reads a list of waypoints from a YAML file, waits until the Nav2
``navigate_through_poses`` action server is available, and then sends the
*entire* list of waypoints as a **single** ``NavigateThroughPoses`` goal.

Unlike sending one ``NavigateToPose`` goal per waypoint, this lets Nav2 plan a
**single global path through all the waypoints** and drive it without stopping
at each intermediate pose.  The action only returns a result once the whole
route has been completed (or has failed), so there is no per-waypoint
start/stop overhead and no artificial delay between waypoints - the route is
driven in the shortest possible way.

Because the navigation is delegated to Nav2, the global planner computes the
path and publishes it on the ``/plan`` topic (and the local controller on
``/local_plan``).  Those topics are already configured as Path displays in the
TurtleBot3 RViz configuration, so the planned path becomes visible on the map
automatically while the robot drives the route.
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw):
    """Convert a planar yaw angle (radians) into a quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _number(value):
    """Coerce a value to float, tolerating ``None``."""
    return float(value) if value is not None else 0.0


class WaypointFollower(Node):
    """Send the whole waypoint list to Nav2 as one NavigateThroughPoses goal."""

    def __init__(self):
        super().__init__('waypoint_follower')
        self.get_logger().info('=== NEW NavigateThroughPoses node loaded ===')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        default_wp_file = os.path.join(
            get_package_share_directory('turtlebot3_wp_following'),
            'config', 'wp_following.yaml')

        self.declare_parameter('waypoints_file', default_wp_file)
        self.declare_parameter('frame_id', 'map')
        # If true the node loops the whole route forever.
        self.declare_parameter('loop', False)
        # Kept for backwards compatibility with the launch file / older config.
        # It is no longer used internally: the whole route is a single goal, so
        # there is nothing to wait for between waypoints (no extra 2 s wait).
        self.declare_parameter('goal_delay', 0.0)
        # Seconds to sleep between startup retries while waiting for Nav2.
        self.declare_parameter('startup_delay', 5.0)

        self._waypoints_file = self.get_parameter(
            'waypoints_file').get_parameter_value().string_value
        self._frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self._loop = self.get_parameter('loop').get_parameter_value().bool_value
        self._startup_delay = self.get_parameter(
            'startup_delay').get_parameter_value().double_value

        # ------------------------------------------------------------------
        # Waypoints
        # ------------------------------------------------------------------
        self.waypoints = self._load_waypoints(self._waypoints_file)
        if not self.waypoints:
            self.get_logger().error(
                f'No waypoints loaded from "{self._waypoints_file}". '
                'Nothing to do.')
        else:
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints from '
                f'"{self._waypoints_file}".')

        # ------------------------------------------------------------------
        # Nav2 action client (NavigateThroughPoses navigator)
        # ------------------------------------------------------------------
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')

        self._goal_in_progress = False
        self._waiting_for_server = True

        # Wait for the Nav2 action server, then send the single route goal.
        self._startup_timer = self.create_timer(
            self._startup_delay, self._try_start)

        self.get_logger().info(
            'Waiting for the Nav2 "navigate_through_poses" action server ...')

    # ----------------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------------
    def _load_waypoints(self, path):
        """Read waypoints from a YAML file.

        The primary supported format matches the one produced by the RViz
        "Publish Point"-style waypoint saver used for this project::

            waypoints:
              waypoint0:
                pose: [1.49, -0.01, 0.0]      # x, y, z
                orientation: [w, x, y, z]     # quaternion (w first!)
              waypoint1:
                pose: [...]

        Also accepts a plain list of dictionaries/lists for convenience::

            waypoints:
              - {x: 1.0, y: 0.5, yaw: 0.0}
              - [2.0, 1.5, 1.57]
        """
        if not os.path.isfile(path):
            self.get_logger().warn(f'Waypoint file not found: {path}')
            return []

        with open(path, 'r') as handle:
            data = yaml.safe_load(handle) or {}

        raw = data.get('waypoints', data) if isinstance(data, dict) else data
        entries = []

        if isinstance(raw, dict):
            # {"waypoint0": {...}, "waypoint1": {...}} - sort by the numeric
            # suffix so the order does not depend on YAML key ordering.
            def _key(name):
                digits = ''.join(c for c in str(name) if c.isdigit())
                return int(digits) if digits else 0
            entries = [raw[k] for k in sorted(raw, key=_key)]
        elif isinstance(raw, list):
            entries = raw
        else:
            self.get_logger().error(
                'Waypoint file must contain a "waypoints" mapping or list.')
            return []

        waypoints = []
        for entry in entries:
            wp = self._parse_entry(entry)
            if wp is not None:
                waypoints.append(wp)
        return waypoints

    def _parse_entry(self, entry):
        """Parse a single waypoint entry into an internal dict.

        Returns ``None`` when the entry cannot be understood.
        """
        frame = self._frame_id

        # Format A: {"pose": [x, y, z], "orientation": [w, x, y, z]}
        if isinstance(entry, dict) and 'pose' in entry and 'orientation' in entry:
            pose = entry['pose']
            ori = entry['orientation']
            if not isinstance(pose, (list, tuple)) or len(pose) < 2:
                return None
            if not isinstance(ori, (list, tuple)) or len(ori) != 4:
                return None
            x = _number(pose[0])
            y = _number(pose[1])
            # Orientation is stored as [w, x, y, z]; reorder to ROS x, y, z, w.
            qw, qx, qy, qz = (_number(v) for v in ori)
            return {
                'x': x, 'y': y,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                'frame_id': entry.get('frame_id', frame),
            }

        # Format B: {"x": .., "y": .., "yaw"/"theta": ..}
        if isinstance(entry, dict):
            x = _number(entry.get('x'))
            y = _number(entry.get('y'))
            yaw = _number(entry.get('yaw', entry.get('theta', 0.0)))
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            return {
                'x': x, 'y': y,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                'frame_id': entry.get('frame_id', frame),
            }

        # Format C: [x, y, yaw]
        if isinstance(entry, (list, tuple)) and len(entry) >= 2:
            x = _number(entry[0])
            y = _number(entry[1])
            yaw = _number(entry[2]) if len(entry) > 2 else 0.0
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            return {
                'x': x, 'y': y,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                'frame_id': frame,
            }

        return None

    def _try_start(self):
        """Poll until the Nav2 action server is ready, then send the route goal."""
        if self._action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().info(
                'Nav2 "navigate_through_poses" action server is available.')
            self._startup_timer.cancel()
            self._waiting_for_server = False
            self._send_route()

    # ----------------------------------------------------------------------
    # Navigation
    # ----------------------------------------------------------------------
    def _pose_to_msg(self, waypoint):
        """Convert an internal waypoint dict into a stamped PoseStamped."""
        pose = PoseStamped()
        pose.header.frame_id = waypoint['frame_id']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint['x']
        pose.pose.position.y = waypoint['y']
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = waypoint['qx']
        pose.pose.orientation.y = waypoint['qy']
        pose.pose.orientation.z = waypoint['qz']
        pose.pose.orientation.w = waypoint['qw']
        return pose

    def _make_goal(self):
        """Build one NavigateThroughPoses goal holding every waypoint."""
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._pose_to_msg(wp) for wp in self.waypoints]
        return goal

    def _send_route(self):
        if self._goal_in_progress or self._waiting_for_server:
            return

        if not self.waypoints:
            return

        self.get_logger().info(
            f'Sending a single route through {len(self.waypoints)} waypoints '
            'via NavigateThroughPoses (one global path, no intermediate stops).')

        self._goal_in_progress = True
        goal = self._make_goal()
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        send_future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Newer distros report the number of poses still to visit.
        remaining = getattr(feedback, 'number_of_poses_remaining', None)
        if remaining is not None:
            self.get_logger().debug(
                f'Route in progress: {remaining} poses remaining.')
        else:
            # Older distros expose distance_remaining (< 0 when unknown).
            dist = getattr(feedback, 'distance_remaining', -1.0)
            if dist >= 0.0:
                self.get_logger().debug(f'Route in progress: {dist:.2f} m left.')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Route goal was rejected by Nav2.')
            self._goal_in_progress = False
            self._on_route_finished()
            return

        self.get_logger().info(
            'Route accepted. Nav2 is planning / driving the whole path - '
            'it is shown on /plan in RViz.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        status = future.result().status
        # GoalStatus.STATUS_SUCCEEDED == 4
        if status == 4:
            self.get_logger().info('Route completed - all waypoints reached.')
        else:
            self.get_logger().warn(
                f'Route finished with status {status} (not SUCCEEDED).')

        self._goal_in_progress = False
        self._on_route_finished()

    def _on_route_finished(self):
        """Called once the whole route is done (no per-waypoint delay)."""
        if self._loop and self.waypoints:
            self.get_logger().info('Looping - sending the route again.')
            self._send_route()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
