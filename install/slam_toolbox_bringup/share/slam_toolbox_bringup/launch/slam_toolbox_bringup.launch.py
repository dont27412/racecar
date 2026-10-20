#!/usr/bin/env python3
"""启动 slam_toolbox 在线异步建图。

功能：
    读取本包 config/slam_toolbox_bringup.yaml，然后调用 slam_toolbox 官方
    online_async_launch.py 启动在线异步建图。

需要用户重点设置的内容：
    1. 建图参数主要在 config/slam_toolbox_bringup.yaml 中修改；
    2. use_sim_time 只决定使用系统时间还是 /clock；
    3. 实体车通常 use_sim_time:=false，仿真/rosbag 回放通常为 true。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """构造 slam_toolbox bringup 的 LaunchDescription。

    功能：
        把本工程的 YAML 参数和 use_sim_time 传给 slam_toolbox 官方启动文件。

    返回：
        LaunchDescription：供 `ros2 launch slam_toolbox_bringup ...` 使用。
    """

    # use_sim_time
    #   功能：运行时读取命令行参数 use_sim_time。
    #   参数作用：false 使用电脑/机器人系统时间；true 使用 /clock。
    #   怎么设置：
    #     实体机器人：false。
    #     Gazebo/仿真/发布 /clock 的 rosbag：true。
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 功能：自动定位两个功能包安装后的 share 目录。
    # 怎么设置：无需填写绝对路径；只要工作空间已 build 并 source 即可。
    bringup_share = get_package_share_directory("slam_toolbox_bringup")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    # slam_params_file
    #   功能：指定 slam_toolbox 实际加载的参数文件。
    #   怎么设置：主要调参都改这个 YAML；如果另建多套配置，也可在此切换文件名。
    slam_params_file = os.path.join(
        bringup_share,
        "config",
        "slam_toolbox_bringup.yaml",
    )

    # 功能：包含 slam_toolbox 官方在线异步建图启动文件。
    # 参数作用：online_async 会持续处理激光和里程计数据，适合实时建图。
    # 怎么设置：正常实时建图保持 online_async_launch.py；除非明确要改成其他官方模式。
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_toolbox_share,
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            # 功能：把本工程 YAML 路径传给 slam_toolbox。
            "slam_params_file": slam_params_file,

            # 功能：把当前 Launch 的时间模式继续传给 slam_toolbox。
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            # 默认值怎么设置：实体车默认 false 最安全；仿真工程可改成 true。
            default_value="false",
            description="Use simulation clock if true.",
        ),
        slam_toolbox_launch,
    ])
