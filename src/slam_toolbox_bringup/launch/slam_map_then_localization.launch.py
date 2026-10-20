#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""建图 + 自动存图 + 自动切换定位（map_and_localization）一键启动。

功能
====
1. 启动 ``map_and_localization_slam_toolbox_node``（初始 mapping 模式）开始建图；
2. 同时启动 ``slam_auto_switch`` 节点，订阅 ``/slam_switch_trigger``；
3. 当惯性导航小车经过预设点并发布该话题后，auto_switch 节点会：
   - 调用 ``save_map`` 保存地图到默认位置（默认 ``~/maps/<map_name>``）；
   - 把 slam_toolbox 的 ``mode`` 参数运行时切换为 ``localization``，
     即从"建图"直接切为"定位"，无需关闭 slam_toolbox、无需启动 AMCL。

这与官方 ``online_async_launch.py`` 的关键区别
=============================================
官方 online_async 启动的是 ``async_slam_toolbox_node``，**不支持**运行时切换
mode。本 launch 使用 ``map_and_localization_slam_toolbox_node``，它支持通过
``set_parameters`` 在 mapping / localization 之间切换。

启动方式::

    ros2 launch slam_toolbox_bringup slam_map_then_localization.launch.py

常用参数::

    use_sim_time:=false         实体车用 false，仿真用 true
    map_name:=arena             保存的地图名（不含扩展名）
    save_dir:=/home/user/maps   地图保存目录
    target_mode:=localization   触发后切换到的模式
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('slam_toolbox_bringup')

    # 建图阶段的参数文件（mode: mapping）。
    slam_params_file = os.path.join(
        bringup_share, 'config', 'slam_toolbox_bringup.yaml')
    rviz_config_file = os.path.join(
        bringup_share, 'rviz', 'slam_map_local.rviz')
    use_rviz = LaunchConfiguration('use_rviz')

    # ----------------------------------------------------------------
    # 运行时参数
    # ----------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map_name')
    save_dir = LaunchConfiguration('save_dir')
    target_mode = LaunchConfiguration('target_mode')
    slam_node_name = LaunchConfiguration('slam_node_name')
    trigger_topic = LaunchConfiguration('trigger_topic')

    # ----------------------------------------------------------------
    # 1) slam_toolbox：支持运行时切 mode 的节点
    # ----------------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='map_and_localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ----------------------------------------------------------------
    # 2) 自动存图 + 切模式节点
    # ----------------------------------------------------------------
    auto_switch_node = Node(
        package='slam_toolbox_bringup',
        executable='slam_auto_switch',
        name='slam_auto_switch',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'trigger_topic': trigger_topic,
            'slam_node_name': slam_node_name,
            'save_dir': save_dir,
            'map_name': map_name,
            'target_mode': target_mode,
            'serialize': True,
            'trigger_once': True,
        }],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='实体车 false；仿真 true'),
        DeclareLaunchArgument(
            'map_name', default_value='my_map',
            description='保存的地图名（不含扩展名）'),
        DeclareLaunchArgument(
            'save_dir', default_value=os.path.expanduser('~/maps'),
            description='地图保存目录'),
        DeclareLaunchArgument(
            'target_mode', default_value='localization',
            description='触发后切换到的模式（localization/mapping）'),
        DeclareLaunchArgument(
            'slam_node_name', default_value='/slam_toolbox',
            description='slam_toolbox 节点名（用于拼服务名）'),
        DeclareLaunchArgument(
            'trigger_topic', default_value='/slam_switch_trigger',
            description='触发存图/切模式的话题'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否同时自动打开 rviz2'),

        # ...（原有的 slam_node、auto_switch_node 保持不动）...
        rviz_node,
        slam_node,
        auto_switch_node,
    ])
