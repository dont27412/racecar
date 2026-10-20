# 雷达运动去畸变改动说明

## 已发现的问题

当前 `lslidar_driver/src/lslidar_x10_driver.cpp` 在生成 `/scan` 时：

1. `header.stamp` 使用一圈扫描结束时间 `sweep_end_time`；
2. `time_increment` 被固定为 `0.0`；
3. X10/M10GPS 原始方位角按 0° -> 360° 增长，并在 0° 处换圈；
4. 转为 LaserScan 时使用 `atan2(-y, x)`，因此 LaserScan 的角度顺序不能直接当成时间顺序。

车辆在约 0.1 s 的一圈扫描时间内发生平移/转向时，会出现直墙变弯、边缘重影等运动畸变。

## 本次修改

新增 `laser_deskew` 包：

- 订阅 `/scan`；
- 订阅 EKF 输出 `/odom_combined`；
- 根据 X10 驱动的 0° 换圈逻辑和 `atan2(-y, x)` 映射，重建每束激光的采样相位；
- 对 `/odom_combined` 的 x/y/yaw 做时间插值；
- 使用运动学参考点 `akm_rear_axle_footprint -> laser_link` 的平面外参 `(x=0.25, y=0, yaw=0)`；
- 将每束激光变换到该圈结束时刻的雷达坐标系；
- 重新栅格化并发布 `/scan_deskewed`；
- `slam_toolbox` 已改为订阅 `/scan_deskewed`。

## 启动顺序

```bash
ros2 launch lslidar_driver lslidar_x10_launch.py
ros2 launch ekf_bringup ekf_bringup.launch.py
ros2 launch function_py robot_tf.launch.py
ros2 launch laser_deskew deskew.launch.py
ros2 launch slam_toolbox_bringup slam_toolbox_bringup.launch.py
```

首次加入新包后先：

```bash
colcon build --symlink-install
source install/setup.bash
```

## 验证

```bash
ros2 topic hz /scan
ros2 topic hz /odom_combined
ros2 topic hz /scan_deskewed
```

在 RViz 同时显示 `/scan` 和 `/scan_deskewed`，让车转弯经过直墙。正确时 `/scan_deskewed` 的墙更直、重影更小。

若去畸变后反而更差，把 `laser_deskew/config/deskew.yaml` 中：

```yaml
raw_azimuth_increases: true
```

临时改为 `false` 对比，用于排查具体固件扫描方向是否和当前 M10GPS 解包源码一致。
