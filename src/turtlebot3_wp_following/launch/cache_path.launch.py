"""Cache the Nav2 global path (``/plan``) into a waypoints YAML file.

Run this **together** with a single waypoint-follower run so that Nav2's
planned global path gets captured::

    # terminal 1: gazebo
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

    # terminal 2: nav2
    ros2 launch turtlebot3_navigation2 navigation2.launch.py \
        use_sim_time:=true map:=/home/mlh090300/my_map.yaml

    # terminal 3: the cacher
    ros2 launch turtlebot3_wp_following cache_path.launch.py

    # terminal 4: one waypoint-follower run (this publishes /plan)
    ros2 launch turtlebot3_wp_following wp_following.launch.py

When the follower finishes, ``config/cached_path.yaml`` holds the path.
Afterwards it can be tracked without Nav2::

    ros2 launch turtlebot3_wp_following track_path.launch.py
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    # Prefer the source tree config dir so the cached file survives rebuilds.
    source_config = os.path.join(
        os.path.expanduser('~'),
        'turtlebot3_ws', 'src', 'turtlebot3', 'turtlebot3_wp_following',
        'config')
    if not os.path.isdir(source_config):
        source_config = os.path.expanduser('~')

    plan_topic = LaunchConfiguration('plan_topic')
    output_file = LaunchConfiguration('output_file')
    min_spacing = LaunchConfiguration('min_spacing')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Run the node directly with the Python interpreter instead of relying on
    # the libexec (<prefix>/lib/<pkg>/<exe>) lookup, which can be flaky on some
    # colcon/setuptools setups.  Matches the working turtlebot3_ekf_nav package.
    path_cacher = ExecuteProcess(
        cmd=[
            sys.executable, '-m',
            'turtlebot3_wp_following.path_cacher',
            '--ros-args',
            '-p', ['use_sim_time:=', use_sim_time],
            '-p', ['plan_topic:=', plan_topic],
            '-p', ['output_file:=', output_file],
            '-p', ['min_spacing:=', min_spacing],
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'plan_topic', default_value='/plan',
            description='Path topic published by the Nav2 global planner'),
        DeclareLaunchArgument(
            'output_file',
            default_value=os.path.join(source_config, 'cached_path.yaml'),
            description='Where to write the cached waypoints YAML'),
        DeclareLaunchArgument(
            'min_spacing', default_value='0.20',
            description='Minimum spacing (m) between cached waypoints'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        path_cacher,
    ])

