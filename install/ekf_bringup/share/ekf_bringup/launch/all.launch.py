#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""按 3 秒间隔依次启动整套机器人功能，并验证所有节点实际运行。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include_launch(package_name, launch_file):
    """包含指定功能包的 launch 文件。"""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                "launch",
                launch_file,
            )
        )
    )


def generate_launch_description():
    # 预期启动的 13 个 ROS 2 节点。
    # 连续 3 秒全部存在于 ROS graph，才认为本次启动验证通过。
    expected_nodes = [
        "/rear_axle_to_base_footprint",
        "/base_footprint_to_base_link",
        "/base_link_to_imu_link",
        "/base_link_to_laser_link",
        "/imu_raw_raw",
        "/encoder_raw",
        "/ekf_bringup",
        "/imu_filter_madgwick",
        "/ekf_filter_node",
        "/x10/lslidar_driver_node",
        "/laser_deskew",
        "/slam_toolbox",
        "/racecar_driver",
    ]

    node_array = " ".join(f"'{node}'" for node in expected_nodes)
    health_check_script = f"""
expected=({node_array})
stable=0
end=$((SECONDS + 60))

while (( SECONDS < end )); do
    nodes="$(ros2 node list 2>/dev/null || true)"
    missing=()

    for node in "${{expected[@]}}"; do
        if ! grep -Fxq "$node" <<< "$nodes"; then
            missing+=("$node")
        fi
    done

    if (( ${{#missing[@]}} == 0 )); then
        stable=$((stable + 1))
        if (( stable >= 3 )); then
            exit 0
        fi
    else
        stable=0
        echo "[ekf_bringup] 等待节点: ${{missing[*]}}"
    fi

    sleep 1
done

exit 1
"""

    # 最后一个功能包启动 3 秒后开始验证；最多等待 60 秒。
    # 只有全部预期节点连续 3 秒存在，检查进程才会以 0 退出。
    health_check = ExecuteProcess(
        cmd=["bash", "-c", health_check_script],
        output="screen",
    )

    def health_check_result(event, context):
        if event.returncode == 0:
            return [LogInfo(msg=(
                "\n"
                "==================================================\n"
                "==================================================\n"
                "==========                              ==========\n"
                "==========         系统启动成功         ==========\n"
                "==========                              ==========\n"
                "==================================================\n"
                "=================================================="
            ))]
        return [LogInfo(msg="[ERROR] 功能包启动验证失败：60 秒内仍有节点未正常运行")]

    return LaunchDescription([
        # 0 s：静态 TF
        include_launch("ekf_bringup", "robot_tf.launch.py"),

        # 3 s：IMU、编码器、Madgwick、EKF
        TimerAction(
            period=3.0,
            actions=[include_launch("ekf_bringup", "ekf_bringup.launch.py")],
        ),

        # 6 s：雷达驱动（沿用项目 README 中的 X10 默认启动方式）
        TimerAction(
            period=6.0,
            actions=[include_launch("lslidar_driver", "lslidar_x10_launch.py")],
        ),

        # 9 s：激光去畸变
        TimerAction(
            period=9.0,
            actions=[include_launch("laser_deskew", "deskew.launch.py")],
        ),

        # 12 s：SLAM
        TimerAction(
            period=12.0,
            actions=[include_launch("slam_toolbox_bringup", "slam_toolbox_bringup.launch.py")],
        ),

        # 15 s：底盘串口驱动
        TimerAction(
            period=15.0,
            actions=[
                Node(
                    package="ekf_bringup",
                    executable="racecar_driver",
                    name="racecar_driver",
                    output="screen",
                )
            ],
        ),

        # 18 s：开始验证全部节点是否已经实际运行。
        TimerAction(
            period=18.0,
            actions=[health_check],
        ),

        # 根据验证进程结果打印 INFO 或 ERROR，避免固定时间误报成功。
        RegisterEventHandler(
            OnProcessExit(
                target_action=health_check,
                on_exit=health_check_result,
            )
        ),
    ])
