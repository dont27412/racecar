"""Launch the TurtleBot3 waypoint follower.

Usage (after Nav2 is up)::

    ros2 launch turtlebot3_navigation2 navigation2.launch.py \
        use_sim_time:=true map:=/home/mlh090300/my_map.yaml

    ros2 launch turtlebot3_wp_following wp_following.launch.py

The follower waits for the Nav2 ``navigate_through_poses`` action server and
then sends the entire waypoint list from ``wp_following.yaml`` as a single
``NavigateThroughPoses`` goal: Nav2 plans one global path through all the
waypoints and drives it without stopping in between. The planned path shows up
on the RViz map automatically (``/plan`` topic).
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_wp_following')
    default_wp_file = os.path.join(pkg_share, 'config', 'wp_following.yaml')
    # Prefer the user's own waypoint file if it exists.
    user_wp_file = os.path.expanduser('~/wp_following.yaml')
    if os.path.isfile(user_wp_file):
        default_wp_file = user_wp_file

    use_sim_time = LaunchConfiguration('use_sim_time')
    waypoints_file = LaunchConfiguration('waypoints_file')
    loop = LaunchConfiguration('loop')

    # Run the node directly with the Python interpreter instead of relying on
    # the libexec executable lookup (install/<pkg>/lib/<pkg>/<exe>).  This
    # avoids the "libexec directory ... does not exist" launch error that can
    # happen with ament_python packages, and matches the approach used by the
    # turtlebot3_ekf_nav package in this workspace.
    #
    # ExecuteProcess does not forward declared arguments automatically, so the
    # values are substituted into the command line explicitly, including
    # use_sim_time (so this node uses the Gazebo clock, not the wall clock).
    waypoint_follower = ExecuteProcess(
        cmd=[
            sys.executable, '-m',
            'turtlebot3_wp_following.waypoint_follower',
            '--ros-args',
            '-p', ['use_sim_time:=', use_sim_time],
            '-p', ['waypoints_file:=', waypoints_file],
            '-p', ['frame_id:=map'],
            '-p', ['loop:=', loop],
            '-p', ['startup_delay:=5.0'],
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'waypoints_file', default_value=default_wp_file,
            description='Full path to the waypoints YAML file'),
        DeclareLaunchArgument(
            'loop', default_value='false',
            description='Loop the whole route forever when it completes'),
        waypoint_follower,
    ])

