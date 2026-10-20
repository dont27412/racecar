#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""启动 ekf_bringup 整套 2D 融合链路。

数据流：
  CH100 -> imu_raw_raw -> /imu_raw_raw
                             |
                             v
                       ekf_bringup -> /imu_raw -> imu_filter_madgwick -> /imu/data
                             |
  encoder -> encoder_raw -> /encoder_raw
                             |
                             v
                           /odom

  robot_localization EKF:
    odom0 = /odom，只融合 vx、vy
    imu0  = /imu/data，融合 yaw 和 angular_velocity.z
    输出  = /odom_combined，并发布 odom -> akm_rear_axle_footprint TF

说明：
- /imu_raw 是已经做过陀螺仪零飘补偿的六轴 IMU 数据。
- Madgwick 仍接收完整三轴角速度和三轴线加速度；EKF 从 /imu/data 融合 yaw 与 Z 轴角速度。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 通过 ament 索引获取安装后的功能包 share 目录。
    # 原文件使用了未定义的 bringup_dir，会在 launch 解析阶段直接报错。
    package_share_dir = get_package_share_directory("ekf_bringup")
    ekf_config = os.path.join(package_share_dir, "config", "ekf.yaml")
    imu_config = os.path.join(package_share_dir, "config", "imu.yaml")

    # ------------------------------------------------------------
    # 1. 原始传感器节点
    # ------------------------------------------------------------
    # CH100 原始六轴 IMU：发布 /imu_raw_raw，sensor_msgs/msg/Imu，约 100 Hz。
    imu_raw_raw_node = Node(
        package="ekf_bringup",
        executable="imu_raw_raw",
        name="imu_raw_raw",
        output="screen",
    )

    # 编码器：发布 /encoder_raw，geometry_msgs/msg/Vector3Stamped，50 Hz。
    # vector.x/y/z 分别表示 Vx/Vy/Vz；当前驱动中 Vy=Vz=0。
    encoder_raw_node = Node(
        package="ekf_bringup",
        executable="encoder_raw",
        name="encoder_raw",
        output="screen",
    )

    # ------------------------------------------------------------
    # 2. EKF 前处理 / 零飘校准节点
    # ------------------------------------------------------------
    # 订阅 /imu_raw_raw 和 /encoder_raw：
    # - 100 Hz 发布 /imu_raw，并对陀螺仪 Gx/Gy/Gz 做零飘补偿；
    # - 50 Hz 发布 /odom，只提供编码器 vx、vy；
    # - 完成首次与自动陀螺仪零飘校准。
    ekf_bringup_node = Node(
        package="ekf_bringup",
        executable="ekf_bringup",
        name="ekf_bringup",
        output="screen",
    )

    # ------------------------------------------------------------
    # 3. TF 由本功能包的 robot_tf.launch.py 发布
    # ------------------------------------------------------------
    # 本 launch 不再发布任何静态 TF。
    # 如需静态 TF，请单独启动：ros2 launch ekf_bringup robot_tf.launch.py

    # ------------------------------------------------------------
    # 4. Madgwick 姿态滤波
    # ------------------------------------------------------------
    # imu_filter_madgwick 默认订阅 imu/data_raw、发布 imu/data。
    # 这里把输入显式重映射到 ekf_bringup 发布的 /imu_raw。
    # use_mag=false，因此不需要磁力计话题。
    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[imu_config, {"use_sim_time": False}],
        remappings=[
            ("imu/data_raw", "/imu_raw"),
            ("imu/data", "/imu/data"),
        ],
    )

    # ------------------------------------------------------------
    # 5. robot_localization 2D EKF
    # ------------------------------------------------------------
    # ekf.yaml 中：
    # - odom0=/odom，只打开 vx、vy；
    # - imu0=/imu/data，打开 yaw（orientation 绕 Z 轴）和 vyaw（angular_velocity.z）；
    # - two_d_mode=true；
    # - EKF 发布 odom -> akm_rear_axle_footprint TF。
    robot_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": False}],
        remappings=[
            ("odometry/filtered", "/odom_combined"),
        ],
    )

    return LaunchDescription([
        imu_raw_raw_node,
        encoder_raw_node,
        ekf_bringup_node,
        imu_filter_node,
        robot_ekf,
    ])
