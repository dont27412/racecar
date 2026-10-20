"""Track a cached path with the Nav2-free pure-pursuit tracker.

Reads the waypoints YAML written by ``cache_path.launch.py`` / ``path_cacher``
and drives the robot along it using pure pursuit.  **Nav2 is not required** -
only TF (``map`` -> ``base_link``) and ``cmd_vel`` need to exist.

Usage::

    ros2 launch turtlebot3_wp_following track_path.launch.py

    # or point at a specific file:
    ros2 launch turtlebot3_wp_following track_path.launch.py \
        path_file:=/home/mlh090300/cached_path.yaml
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    source_config = os.path.join(
        os.path.expanduser('~'),
        'turtlebot3_ws', 'src', 'turtlebot3', 'turtlebot3_wp_following',
        'config')
    if not os.path.isdir(source_config):
        source_config = os.path.expanduser('~')

    path_file = LaunchConfiguration('path_file')
    lookahead = LaunchConfiguration('lookahead')
    max_linear = LaunchConfiguration('max_linear')
    loop = LaunchConfiguration('loop')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Run the node directly with the Python interpreter (see cache_path.launch.py
    # for why we avoid the libexec lookup).
    path_tracker = ExecuteProcess(
        cmd=[
            sys.executable, '-m',
            'turtlebot3_wp_following.path_tracker',
            '--ros-args',
            '-p', ['use_sim_time:=', use_sim_time],
            '-p', ['path_file:=', path_file],
            '-p', ['lookahead:=', lookahead],
            '-p', ['max_linear:=', max_linear],
            '-p', ['loop:=', loop],
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'path_file',
            default_value=os.path.join(source_config, 'cached_path.yaml'),
            description='Cached waypoints YAML to track'),
        DeclareLaunchArgument(
            'lookahead', default_value='0.35',
            description='Pure-pursuit lookahead distance (m)'),
        DeclareLaunchArgument(
            'max_linear', default_value='0.22',
            description='Maximum linear speed (m/s)'),
        DeclareLaunchArgument(
            'loop', default_value='false',
            description='Loop the tracked path forever'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        path_tracker,
    ])

