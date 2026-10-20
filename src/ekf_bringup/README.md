# ekf_bringup

ROS 2 Python 功能包，用于小车 IMU/编码器前处理、陀螺仪零飘校准、Madgwick 姿态滤波和 `robot_localization` 2D EKF 融合。

## 数据链路

```text
CH100 -> imu_raw_raw -> /imu_raw_raw (sensor_msgs/msg/Imu, ~100 Hz)
                              |
                              v
                        ekf_bringup -> /imu_raw (sensor_msgs/msg/Imu, 100 Hz)
                              |             |
                              |             v
encoder -> encoder_raw -> /encoder_raw   imu_filter_madgwick -> /imu/data
                              |
                              v
                            /odom (nav_msgs/msg/Odometry, 50 Hz)

robot_localization EKF:
  odom0 = /odom      -> 只融合 vx、vy
  imu0  = /imu/data  -> 融合 yaw 和 angular_velocity.z
  output = /odom_combined -> 30 Hz
```

## 功能包内容

- `ekf_bringup/imu_raw_raw.py`：CH100 串口驱动，发布 `/imu_raw_raw`。
- `ekf_bringup/encoder_raw.py`：编码器 Modbus RTU 驱动，发布 `/encoder_raw`。
- `/imu_raw_raw` 与 `/encoder_raw` 的原始消息不设置 `header.frame_id`；只有 `ekf_bringup` 输出时，IMU 使用 `imu_link`，编码器里程计使用 `akm_rear_axle_footprint`。
- `ekf_bringup/ekf_bringup.py`：静止判定、陀螺仪零飘校准、发布 `/imu_raw` 和 `/odom`。
- `config/imu.yaml`：`imu_filter_madgwick` 参数。
- `config/ekf.yaml`：`robot_localization` 2D EKF 参数。
- `launch/ekf_bringup.launch.py`：一次启动 IMU、编码器、Madgwick 与 robot_localization EKF。
- `launch/robot_tf.launch.py`：发布小车静态 TF。
- `launch/all.launch.py`：按原先顺序启动 TF、EKF、雷达、去畸变、SLAM 和底盘驱动。
- `ekf_bringup/racecar_driver.py`：底盘串口驱动，订阅 `/cmd_vel`。

## 零飘校准规则

首次校准：

1. 小车连续静止 10 s。
2. 进入 10 s 陀螺仪原始角速度采集。
3. 对 Gx/Gy/Gz 求平均，作为新的零飘偏置。
4. 采集途中移动则丢弃本次样本，并重新等待连续静止 10 s。

自动校准：

1. 只有首次校准成功后才启用。
2. 每次校准完成后 30 s 内不再次进入采集。
3. 之后再次连续静止 10 s，再采集 10 s。
4. 自动采集中移动时，放弃本轮采集，继续使用上一组有效偏置。

静止条件：

- 编码器：`|Vx|, |Vy|, |Vz| < 0.02 m/s`，并且相邻样本变化量均 `< 0.02 m/s`。
- IMU 角速度：校正后的 `|Gx|, |Gy|, |Gz| < 0.02 rad/s`，并且变化量均 `< 0.02 rad/s`。
- IMU 加速度：只检查相邻样本 `|dAx|, |dAy|, |dAz| < 2.00 m/s²`。不检查加速度绝对值，因为静止时仍包含约 9.8 m/s² 重力。

## 编译

把整个 `ekf_bringup` 文件夹放进 ROS 2 工作空间 `src/`：

```bash
cd ~/ros2_ws
colcon build --packages-select ekf_bringup --symlink-install
source install/setup.bash
```

系统还需要安装运行依赖，例如：

```bash
sudo apt install python3-serial
sudo apt install ros-$ROS_DISTRO-imu-filter-madgwick
sudo apt install ros-$ROS_DISTRO-robot-localization
```

## 启动

启动整套链路：

```bash
ros2 launch ekf_bringup ekf_bringup.launch.py
```

启动整车功能：

```bash
ros2 launch ekf_bringup all.launch.py
```

也可以单独运行：

```bash
ros2 run ekf_bringup imu_raw_raw
ros2 run ekf_bringup encoder_raw
ros2 run ekf_bringup ekf_bringup
ros2 run ekf_bringup racecar_driver
```

## 主要话题

```text
/imu_raw_raw   sensor_msgs/msg/Imu
/encoder_raw   geometry_msgs/msg/Vector3Stamped
/imu_raw       sensor_msgs/msg/Imu
/odom          nav_msgs/msg/Odometry
/imu/data      sensor_msgs/msg/Imu
/odom_combined nav_msgs/msg/Odometry
```

## TF

`ekf_bringup.launch.py` 本身不发布静态 TF。当前由 `robot_localization` 动态发布：

```text
odom -> akm_rear_axle_footprint
```

小车的静态 TF 已整合到本功能包的 `robot_tf.launch.py`，由它统一发布：

```text
akm_rear_axle_footprint -> base_footprint
base_footprint -> base_link
base_link -> imu_link
base_link -> laser_link
```

请另外启动：

```bash
ros2 launch ekf_bringup robot_tf.launch.py
```

## 注意

- `/dev/imu` 与 `/dev/encoder` 必须存在且当前用户有串口访问权限。
- `imu_raw_raw.py` 已把 CH100 安装轴转换为车体 XYZ，并转换为 SI 单位；`ekf_bringup.py` 不再次做轴旋转或单位换算。
- 当前 EKF 从 `/imu/data` 融合 **yaw（绕 Z 轴角度）** 和 **`angular_velocity.z`（Z 轴角速度）**；`ax/ay/az` 是线加速度，本配置不把它们作为 EKF 测量量。Madgwick 提供 yaw 姿态。
- 协方差不能单独“关闭” robot_localization 的某个状态；实际融合维度由 `ekf.yaml` 中的 `odom0_config` 和 `imu0_config` 决定。

## EKF 时间戳同步说明

融合链中的时间戳现在按“测量时刻”而不是“发布时刻”处理：

- IMU：CH100 设备毫秒计时先保留真实采样间隔，再持续映射到主机 `SYSTEM_TIME`。驱动使用最近约 2 s 的低延迟样本修正设备晶振与主机时钟的长期漂移，并扣除一帧 0x91 数据在 460800 baud、8N1 下的理论线传输时间。重复/倒退设备时间戳不会发布。
- 编码器：同样使用主机 `SYSTEM_TIME`。时间戳在速度寄存器读取完成时建立，并回退半个 20 ms 速度采样窗，表示该平均速度的采样窗中心；随后读取位置寄存器所花的 Modbus 时间不会再污染速度时间戳。
- `ekf_bringup`：不重新打时间戳，只透传上游测量时间；同时拒绝 0/非递增时间戳，并在 IMU 与编码器最新测量时间持续相差超过 50 ms 时报警。
- `imu_filter_madgwick`：默认 `constant_dt=0`，会使用输入消息 header 计算 `dt`，并把输入 IMU 消息复制为输出后再修改姿态，因此 `/imu/data` 保留 `/imu_raw` 的 header 时间戳。

这仍属于软件时间同步，不是硬件触发同步。编码器本身没有独立硬件时间戳，因此其 20 ms 内部测速窗与 ROS 查询周期之间仍可能存在固定相位误差；若需要亚毫秒级同步，需要 IMU/编码器共享硬件时钟或触发线。

运行时建议关注 `ekf_bringup` 的时间戳报警。正常情况下，不应持续出现“时间戳未递增”“测量时间位于未来”或“IMU/编码器最新测量时间相差 > 50 ms”。
