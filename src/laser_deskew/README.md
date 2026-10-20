# laser_deskew

本功能包用于给旋转式 2D 激光雷达做**运动去畸变（motion deskew）**。

## 这个功能是做什么的

旋转式雷达不是在同一个瞬间得到整圈点，而是在一段时间内逐束扫描。例如 10 Hz 雷达一圈大约需要 0.10 s。如果车辆在这 0.10 s 内发生了平移或转向，同一帧 `/scan` 中不同角度的点其实对应不同的机器人位姿，直墙就可能被“拉弯”。

本节点会根据每束激光的采样时刻和 `/odom_combined` 中的车辆运动，把整圈点统一补偿到同一个参考时刻，然后发布 `/scan_deskewed`。

数据流：

`/scan + /odom_combined -> laser_deskew -> /scan_deskewed -> SLAM`

## 启动

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch laser_deskew deskew.launch.py
```

## 参数怎么设置

详细说明已经直接写在 `config/deskew.yaml` 中。每个参数都包含：**功能、参数作用、怎么设置**。

最重要的参数是：

- `laser_x / laser_y / laser_yaw`：从 URDF/TF 中读取雷达相对底盘的真实安装外参，不要凭感觉填写。
- `scan_duration`：近似等于 `1 / 激光扫描频率`。例如 10 Hz 设置约 `0.10`。
- `stamp_is_end`：查看雷达驱动源码，确认 `LaserScan.header.stamp` 代表扫描开始还是结束。当前 X10 驱动按结束时间使用，因此为 `true`。
- `raw_azimuth_increases`：运行 `check_scan_direction.py` 判断；脚本会输出应设为 `true` 还是 `false`。
- `max_extrapolation`：只用于容忍少量 scan/odom 时间抖动，建议从 `0.01~0.05 s` 范围内的小值开始，不要用很大的值掩盖时间同步问题。

## 怎么判断调对了

先检查频率：

```bash
ros2 topic hz /scan
ros2 topic hz /odom_combined
ros2 topic hz /scan_deskewed
```

然后在 RViz 同时显示 `/scan` 和 `/scan_deskewed`，让车辆以明显角速度经过直墙：

- 参数正确：`/scan_deskewed` 中墙线应比 `/scan` 更直，转弯时重影减小。
- 结果更差：优先检查 `laser_x/y/yaw`、`stamp_is_end`、`raw_azimuth_increases`。
- 偶发丢帧：再检查 `odom_history_seconds`、`max_extrapolation` 和两个话题时间戳是否一致。
