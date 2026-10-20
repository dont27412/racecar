# robot_tf

小车静态 TF 功能包。坐标约定采用 ROS REP-103：X 向前、Y 向左、Z 向上。

## TF 树

```text
odom                           # robot_localization 动态发布
└── akm_rear_axle_footprint      # AKM 运动学参考点：后轴中心在地面的投影
    └── base_footprint           (0.17, 0, 0)
        └── base_link            (0, 0, 0.14)
            ├── imu_link       (0.10, 0.035, 0.04)
            └── laser_link     (0.08, 0, 0.12)
```

位置依据：

- `akm_rear_axle_footprint` 是 AKM 2D 运动学参考点，即后轴中心在地面的投影；`base_footprint` 是几何中心在地面的投影，两者 X 方向相差 0.17 m。
- 小车尺寸为长 0.48 m、宽 0.35 m、高 0.28 m，`base_link` 仅表示车体几何中心，不作为 AKM 运动学参考点；因此 `base_footprint -> base_link` 的 Z = 0.28 / 2 = 0.14 m。
- 雷达中心距离车头 0.16 m、位于车体中轴线、离地 0.26 m。`base_link`（几何中心）距离车头 0.24 m，因此相对 `base_link`：X = 0.24 - 0.16 = 0.08 m，Y = 0，Z = 0.26 - 0.14 = 0.12 m。
- IMU 中心距离车头 0.14 m、距离车左边 0.14 m、离地 0.18 m。车体半宽为 0.175 m，因此相对 `base_link`：X = 0.24 - 0.14 = 0.10 m，Y = 0.175 - 0.14 = 0.035 m，Z = 0.18 - 0.14 = 0.04 m。
- 未提供雷达或 IMU 的安装姿态角，因此静态 TF 的 roll/pitch/yaw 暂设为 0。

## 编译

将 `robot_tf` 放入 ROS 2 工作空间的 `src/` 下：

```bash
cd ~/ros2_ws
colcon build --packages-select robot_tf
source install/setup.bash
```

## 启动

```bash
ros2 launch robot_tf robot_tf.launch.py
```

检查：

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo base_footprint base_link
```
