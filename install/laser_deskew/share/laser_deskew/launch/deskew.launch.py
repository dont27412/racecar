#!/usr/bin/env python3
"""启动 laser_deskew 去畸变节点。

功能：
    启动 laser_deskew 节点，并加载 config/deskew.yaml 中的全部参数。

参数怎么改：
    雷达话题、外参、扫描周期、时间戳定义等都在 config/deskew.yaml 修改；
    一般不需要改这个 Launch 文件。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """构造 laser_deskew 的启动描述。

    功能：找到安装后的参数文件，然后启动 ROS 2 节点。

    返回：
        LaunchDescription：供 `ros2 launch laser_deskew deskew.launch.py` 使用。
    """

    # 功能：找到安装后的 laser_deskew 功能包 share 目录。
    # 怎么设置：不需要手动填写路径；ROS 2 会根据已 source 的工作空间自动查找。
    share = get_package_share_directory("laser_deskew")

    # 功能：指定默认参数文件。
    # 怎么设置：正常情况下保持 config/deskew.yaml；如果要维护多套配置，
    #           可以复制不同 YAML，再把这里改成对应文件名。
    config = os.path.join(share, "config", "deskew.yaml")

    return LaunchDescription([
        Node(
            # 功能：指定要启动的 ROS 2 功能包。
            # 怎么设置：包名固定为 laser_deskew，不应随意修改。
            package="laser_deskew",

            # 功能：指定 setup.py 中注册的 console_scripts 可执行入口。
            # 怎么设置：保持 laser_deskew；只有同步修改 setup.py 入口名时才改。
            executable="laser_deskew",

            # 功能：设置 ROS 节点名称。
            # 怎么设置：单实例运行保持 laser_deskew 即可；同时启动多个实例时需使用不同名称/命名空间。
            name="laser_deskew",

            # 功能：把节点日志直接打印到终端。
            # 怎么设置：调试阶段推荐 screen；通常无需修改。
            output="screen",

            # 功能：加载 deskew.yaml，并指定是否使用仿真时间。
            # use_sim_time 怎么设置：
            #   - 实体机器人：False。
            #   - Gazebo 或 rosbag 且系统发布 /clock：True。
            #   - scan 和 odom 必须使用与这里一致的时间基准。
            parameters=[config, {"use_sim_time": False}],
        )
    ])
