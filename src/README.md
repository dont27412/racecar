ros2 launch ekf_bringup all.launch.py









按 3 秒间隔依次启动：TF → EKF → X10 雷达 → 激光去畸变 → SLAM



ros2 launch ekf_bringup robot_tf.launch.py
ros2 launch ekf_bringup ekf_bringup.launch.py
ros2 launch lslidar_driver lslidar_x10_launch.py
ros2 launch laser_deskew deskew.launch.py
ros2 launch slam_toolbox_bringup slam_toolbox_bringup.launch.py

1	TF	机器人各坐标系（关节）的坐标变换
2	EKF	扩展卡尔曼滤波，融合轮式里程计 + IMU，输出更准的位姿
3	X10 雷达	镭神 LS 系列 X10 激光雷达驱动
4	激光去畸变	去除机器人运动导致的激光点云畸变（deskew）
5	SLAM	建图（slam_toolbox 在线建图）







ros2 run tf2_ros tf2_echo map base_footprint
打印 map（地图坐标系）到 base_footprint（机器人底盘在地面的投影点）之间的实时坐标变换