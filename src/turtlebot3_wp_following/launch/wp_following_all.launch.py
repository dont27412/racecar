"""一键启动：仿真 + Nav2 + 路点跟随（waypoint follower）。

等价于原来手动开三个终端依次执行：

终端1 仿真::

    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

终端3 Nav2::

    ros2 launch turtlebot3_navigation2 navigation2.launch.py \
        use_sim_time:=true \
        map:=/home/mlh090300/my_map.yaml \
        params_file:=/home/mlh090300/turtlebot3_ws/install/turtlebot3_wp_following/share/turtlebot3_wp_following/config/nav2_params.yaml

终端2 路点跟随::

    ros2 launch turtlebot3_wp_following wp_following.launch.py

现在只要一条命令::

    ros2 launch turtlebot3_wp_following wp_following_all.launch.py

说明：
* 车型写死为 ``waffle``（与 mapping_auto.launch.py 保持一致），
  并对 Gazebo Classic 的 ``GAZEBO_MODEL_PATH`` 与 Gazebo Sim 的
  ``GZ_SIM_RESOURCE_PATH`` 都做了设置，双保险；
* Nav2 需要先起来、AMCL 定位好，路点跟随才有 action server 可连，
  所以路点跟随被延时启动；
* 启动后请在 RViz 里用 **2D Pose Estimate** 给机器人一个初始位姿，
  AMCL 才能定位，然后路点跟随会自动开始导航。

可选参数::

    use_sim_time:=true                     使用仿真时钟
    map:=<path>                            地图 yaml（默认 ~/my_map.yaml）
    params_file:=<path>                    Nav2 参数（默认本包 config/nav2_params.yaml）
    waypoints_file:=<path>                 路点 yaml（默认与 wp_following.launch.py
                                           一致：若 ~/wp_following.yaml 存在则用它，
                                           否则用本包 config/wp_following.yaml）
    use_rviz:=true                         是否打开 RViz
    loop:=false                            是否循环走完整条路线
    nav2_delay:=12.0                       多少秒后启动 Nav2
    follower_delay:=30.0                   多少秒后启动路点跟随
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ------------------------------------------------------------------
    # 0a. 车型写死为 waffle
    # ------------------------------------------------------------------
    # 必须在“任何被 include 的 launch 的 generate_launch_description() 被
    # 调用之前”就设好环境变量：turtlebot3_gazebo / turtlebot3_navigation2
    # 会在它们自己的 generate_launch_description() 里直接读取
    # os.environ['TURTLEBOT3_MODEL']，而它们在本文件构造
    # IncludeLaunchDescription 时就会被导入并执行。直接给 os.environ
    # 赋值最可靠，避免出现 turtlebot3_burger.urdf 的竞态。
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    loop = LaunchConfiguration('loop')
    nav2_delay = LaunchConfiguration('nav2_delay')
    follower_delay = LaunchConfiguration('follower_delay')

    wp_share = get_package_share_directory('turtlebot3_wp_following')
    gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    # 与 wp_following.launch.py 保持完全一致的默认路点文件选择逻辑：
    #   包内 config/wp_following.yaml，但若家目录存在 ~/wp_following.yaml，
    #   则优先使用家目录那个。这样两条 launch 命令默认读同一个文件。
    _wp_default = os.path.join(wp_share, 'config', 'wp_following.yaml')
    _wp_user = os.path.expanduser('~/wp_following.yaml')
    if os.path.isfile(_wp_user):
        _wp_default = _wp_user

    # ------------------------------------------------------------------
    # 0b. 环境变量：车型 + Gazebo 模型路径
    # ------------------------------------------------------------------
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    _models_local = os.path.join(gazebo_share, 'models')
    _models_system = '/opt/ros/jazzy/share/turtlebot3_gazebo/models'
    _existing_gazebo = os.environ.get('GAZEBO_MODEL_PATH', '')
    _existing_gz_sim = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        ':'.join(p for p in [_models_local, _models_system,
                             _existing_gazebo] if p))
    set_gz_sim_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join(p for p in [_models_local, _models_system,
                             _existing_gz_sim] if p))

    # ------------------------------------------------------------------
    # 1. Gazebo 仿真（对应“终端1”）
    # ------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            gazebo_share, 'launch', 'turtlebot3_world.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ------------------------------------------------------------------
    # 3. Nav2（对应“终端3”）：用地图 + 自定义 nav2 参数
    # ------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'launch', 'navigation2.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. 路点跟随（对应“终端2”）
    # ------------------------------------------------------------------
    follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            wp_share, 'launch', 'wp_following.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'waypoints_file': waypoints_file,
            'loop': loop,
        }.items(),
    )

    return LaunchDescription([
        # -------------------- 参数 ----------------------------------------
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真（Gazebo）时钟'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否打开 RViz（由 navigation2.launch.py 自带）'),
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser('~/my_map.yaml'),
            description='Nav2 使用的地图 yaml 全路径'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(wp_share, 'config', 'nav2_params.yaml'),
            description='Nav2 参数文件全路径'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=_wp_default,
            description='路点 yaml 全路径'),
        DeclareLaunchArgument(
            'loop', default_value='false',
            description='是否循环走完整条路线'),
        DeclareLaunchArgument(
            'nav2_delay', default_value='12.0',
            description='启动后多少秒启动 Nav2'),
        DeclareLaunchArgument(
            'follower_delay', default_value='30.0',
            description='启动后多少秒启动路点跟随（等 Nav2 起来）'),

        # -------------------- 环境 ----------------------------------------
        set_model,
        set_gazebo_model_path,
        set_gz_sim_resource_path,

        # -------------------- 横幅 ----------------------------------------
        LogInfo(msg='========================================================='),
        LogInfo(msg=' TurtleBot3 一键仿真 + Nav2 + 路点跟随'),
        LogInfo(msg='   TURTLEBOT3_MODEL     : waffle'),
        LogInfo(msg=['   use_sim_time         : ', use_sim_time]),
        LogInfo(msg=['   map                  : ', map_file]),
        LogInfo(msg=['   params_file          : ', params_file]),
        LogInfo(msg=['   waypoints_file       : ', waypoints_file]),
        LogInfo(msg='   >>> 启动后在 RViz 里用 2D Pose Estimate 设置初始位姿 <<<'),
        LogInfo(msg='========================================================='),

        # -------------------- 分阶段启动 ----------------------------------
        # 1) Gazebo 最先启动（最重、最慢）。
        TimerAction(period=0.0, actions=[gazebo_launch]),
        # 3) 等仿真起来后启动 Nav2（加载地图、AMCL、planner 等）。
        TimerAction(period=nav2_delay, actions=[nav2_launch]),
        # 2) 等 Nav2 的 action server 就绪后再启动路点跟随。
        TimerAction(period=follower_delay, actions=[follower_launch]),
    ])
