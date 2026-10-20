#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS 2 EKF 前处理节点。

节点名: ekf_bringup

订阅:
  /imu_raw_raw   sensor_msgs/msg/Imu
  /encoder_raw   geometry_msgs/msg/Vector3Stamped

发布:
  /imu_raw       sensor_msgs/msg/Imu      输入一帧发布一帧
  /odom          nav_msgs/msg/Odometry    编码器输入一帧发布一帧

功能:
1. 将上游 IMU 六轴数据重新发布为 EKF/姿态滤波可用的 /imu_raw。
2. 仅对陀螺仪 Gx/Gy/Gz 做零飘估计与补偿；加速度保留重力分量，不做“静止归零”。
3. 使用编码器速度、IMU 角速度和 IMU 加速度变化量联合判断小车静止。
4. 首次零飘校准：连续静止 5 s -> 采集 5 s -> 更新陀螺仪零飘，并重新初始化 EKF。
5. 首次校准成功后开启自动校准；每次校准完成后 30 s 内不再次进入采集。
6. 自动校准采集中若检测到移动，则放弃本次采集并继续使用原零飘值。
7. /odom 只提供 vx、vy；robot_localization 从 /imu/data 融合 yaw 与 Z 轴角速度，IMU 线加速度 ax/ay/az 不参与当前 EKF 测量更新。
"""

import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from robot_localization.srv import SetPose


# ============================================================
# 1. 固定配置
# ============================================================
NODE_NAME = "ekf_bringup"

IMU_INPUT_TOPIC = "/imu_raw_raw"
ENCODER_INPUT_TOPIC = "/encoder_raw"
IMU_OUTPUT_TOPIC = "/imu_raw"
ODOM_OUTPUT_TOPIC = "/odom"

IMU_FRAME_ID = "imu_link"
ODOM_FRAME_ID = "odom"
BASE_FRAME_ID = "akm_rear_axle_footprint"

# 静止判定阈值
VEL_ABS_THRESHOLD = 0.02           # m/s
VEL_DELTA_THRESHOLD = 0.02         # m/s
GYRO_ABS_THRESHOLD = 0.02          # rad/s
GYRO_DELTA_THRESHOLD = 0.02        # rad/s
ACCEL_DELTA_THRESHOLD = 2.00       # m/s^2

# 校准时间
STILL_REQUIRED_S = 5.0            # 连续静止 5 s
CALIBRATION_COLLECT_S = 5.0       # 零飘采集 5 s
CALIBRATION_VALID_S = 30.0         # 完成后 30 s 内不再次进入校准采集

# 输入数据过旧时不继续发布，避免定时器反复把旧样本伪装成新的实时测量。
IMU_STALE_TIMEOUT_S = 0.10
ENCODER_STALE_TIMEOUT_S = 0.20

# 时间戳健康检查。IMU 约 100 Hz、编码器约 50 Hz，正常情况下“最新一帧”
# 的采样时间差通常在几十毫秒以内；超过 50 ms 持续出现时应检查串口延迟
# 或上游时间同步。这里只报警，不强行把两个异步传感器改成相同 timestamp。
TIMESTAMP_ALIGNMENT_WARN_S = 0.05
TIMESTAMP_AGE_WARN_S = 0.20
TIMESTAMP_FUTURE_WARN_S = 0.02

# 协方差
COV_DISABLE = 1e6
GYRO_Z_COV = 1e-3
ACCEL_COV = 1e-3
VX_COV = 1e-4
VY_COV = 1e-3


class EkfControlNode(Node):
    """IMU/编码器前处理、零飘校准与 EKF 输入发布节点。"""

    STATE_WAIT_STILL = "WAIT_STILL"
    STATE_COLLECTING = "COLLECTING"

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        self.info_time_s = 1.0  # INFO/WARN/ERROR 日志节流间隔，单位：秒

        # --------------------------------------------------------
        # 发布器
        # --------------------------------------------------------
        self.imu_publisher = self.create_publisher(Imu, IMU_OUTPUT_TOPIC, 10)
        self.odom_publisher = self.create_publisher(Odometry, ODOM_OUTPUT_TOPIC, 10)

        # --------------------------------------------------------
        # 订阅器
        # --------------------------------------------------------
        self.imu_subscription = self.create_subscription(
            Imu,
            IMU_INPUT_TOPIC,
            self.imu_callback,
            10,
        )
        self.encoder_subscription = self.create_subscription(
            Vector3Stamped,
            ENCODER_INPUT_TOPIC,
            self.encoder_callback,
            10,
        )

        # 首次 IMU 零飘校准完成后，让 robot_localization 从初始状态重新开始融合。
        self.ekf_reset_client = self.create_client(SetPose, "/set_pose")
        self.ekf_reset_done = False

        # 传感器采用事件驱动：收到一帧处理一帧。
        # 不使用定时器重复发布缓存数据，避免伪造时间戳和重复融合。

        # --------------------------------------------------------
        # 最新输入数据
        # --------------------------------------------------------
        self.latest_imu: Optional[Imu] = None
        self.latest_encoder: Optional[Vector3Stamped] = None
        self.latest_imu_receive_s: Optional[float] = None
        self.latest_encoder_receive_s: Optional[float] = None

        # 最近一次已接受的传感器采样时间。用于保证送往 EKF 的每个输入话题
        # 自身严格单调，并监控 IMU/编码器的跨传感器时间偏差。
        self.latest_imu_stamp_ns: Optional[int] = None
        self.latest_encoder_stamp_ns: Optional[int] = None

        # 上一帧，用于计算变化量。
        self.prev_gyro_raw: Optional[Tuple[float, float, float]] = None
        self.prev_accel_raw: Optional[Tuple[float, float, float]] = None
        self.prev_encoder_vel: Optional[Tuple[float, float, float]] = None

        # 当前变化量；None 表示还没有足够的连续样本。
        self.gyro_delta: Optional[Tuple[float, float, float]] = None
        self.accel_delta: Optional[Tuple[float, float, float]] = None
        self.encoder_delta: Optional[Tuple[float, float, float]] = None

        # --------------------------------------------------------
        # 陀螺仪零飘
        # --------------------------------------------------------
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0

        self.first_calibration_done = False
        self.last_calibration_done_s: Optional[float] = None

        self.calibration_state = self.STATE_WAIT_STILL
        self.still_start_s: Optional[float] = None
        self.collect_start_s: Optional[float] = None
        self.collect_sum_x = 0.0
        self.collect_sum_y = 0.0
        self.collect_sum_z = 0.0
        self.collect_count = 0

        self.get_logger().info(
            "ekf_bringup 启动: "
            f"订阅 {IMU_INPUT_TOPIC}, {ENCODER_INPUT_TOPIC}; "
            f"发布 {IMU_OUTPUT_TOPIC}=100Hz, {ODOM_OUTPUT_TOPIC}=50Hz",
            throttle_duration_sec=self.info_time_s,
        )

    # ============================================================
    # 2. 输入回调
    # ============================================================
    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _validate_input_stamp(self, sensor_name: str, stamp_ns: int, last_stamp_ns: Optional[int]) -> bool:
        """拒绝零/倒退 stamp，并对相对主机 SYSTEM_TIME 明显异常的样本报警。"""
        if stamp_ns <= 0:
            self.get_logger().warning(
                f"{sensor_name} 时间戳为 0/非法，已丢弃该帧",
                throttle_duration_sec=self.info_time_s,
            )
            return False

        if last_stamp_ns is not None and stamp_ns <= last_stamp_ns:
            self.get_logger().warning(
                f"{sensor_name} 时间戳未递增: current={stamp_ns}, last={last_stamp_ns}，已丢弃该帧",
                throttle_duration_sec=self.info_time_s,
            )
            return False

        # 上游两路驱动都明确使用 SYSTEM_TIME，因此可直接与 time.time_ns() 比较。
        age_s = (time.time_ns() - stamp_ns) / 1_000_000_000.0
        if age_s > TIMESTAMP_AGE_WARN_S:
            self.get_logger().warning(
                f"{sensor_name} 测量时间比当前系统时间旧 {age_s * 1000.0:.1f} ms",
                throttle_duration_sec=self.info_time_s,
            )
        elif age_s < -TIMESTAMP_FUTURE_WARN_S:
            self.get_logger().warning(
                f"{sensor_name} 测量时间位于系统时间未来 {-age_s * 1000.0:.1f} ms",
                throttle_duration_sec=self.info_time_s,
            )
        return True

    def _check_cross_sensor_stamp_alignment(self) -> None:
        if self.latest_imu_stamp_ns is None or self.latest_encoder_stamp_ns is None:
            return
        skew_s = abs(self.latest_imu_stamp_ns - self.latest_encoder_stamp_ns) / 1_000_000_000.0
        if skew_s > TIMESTAMP_ALIGNMENT_WARN_S:
            self.get_logger().warning(
                f"IMU/编码器最新测量时间相差 {skew_s * 1000.0:.1f} ms；"
                "若持续出现，请检查串口延迟、设备采样周期和系统时钟同步",
                throttle_duration_sec=self.info_time_s,
            )

    def imu_callback(self, msg: Imu) -> None:
        """接收上游六轴 IMU，更新变化量并驱动零飘状态机。"""
        now_s = time.monotonic()
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if not self._validate_input_stamp("IMU", stamp_ns, self.latest_imu_stamp_ns):
            return

        gyro_raw = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )
        accel_raw = (
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
        )

        if not self._all_finite(gyro_raw + accel_raw):
            self.get_logger().warning(
                "收到非有限 IMU 数据，已丢弃该帧",
                throttle_duration_sec=self.info_time_s,
            )
            return

        if self.prev_gyro_raw is not None:
            self.gyro_delta = tuple(
                abs(gyro_raw[i] - self.prev_gyro_raw[i]) for i in range(3)
            )
        else:
            self.gyro_delta = None

        if self.prev_accel_raw is not None:
            self.accel_delta = tuple(
                abs(accel_raw[i] - self.prev_accel_raw[i]) for i in range(3)
            )
        else:
            self.accel_delta = None

        self.prev_gyro_raw = gyro_raw
        self.prev_accel_raw = accel_raw
        self.latest_imu = msg
        self.latest_imu_receive_s = now_s
        self.latest_imu_stamp_ns = stamp_ns
        self._check_cross_sensor_stamp_alignment()

        # 来一帧处理一帧，保留上游原始 timestamp。
        self._update_calibration(now_s, gyro_raw)
        self.Publish_ImuSensor(msg)

    def encoder_callback(self, msg: Vector3Stamped) -> None:
        """接收编码器 Vx/Vy/Vz，计算相邻样本速度变化量。"""
        now_s = time.monotonic()
        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        if not self._validate_input_stamp("编码器", stamp_ns, self.latest_encoder_stamp_ns):
            return

        vel = (
            float(msg.vector.x),
            float(msg.vector.y),
            float(msg.vector.z),
        )

        if not self._all_finite(vel):
            self.get_logger().warning(
                "收到非有限编码器速度，已丢弃该帧",
                throttle_duration_sec=self.info_time_s,
            )
            return

        if self.prev_encoder_vel is not None:
            self.encoder_delta = tuple(
                abs(vel[i] - self.prev_encoder_vel[i]) for i in range(3)
            )
        else:
            self.encoder_delta = None

        self.prev_encoder_vel = vel
        self.latest_encoder = msg
        self.latest_encoder_receive_s = now_s
        self.latest_encoder_stamp_ns = stamp_ns
        self._check_cross_sensor_stamp_alignment()

        # 来一帧处理一帧，保留编码器原始 timestamp。
        self.Publish_Odom(msg)

    # ============================================================
    # 3. 静止判定与零飘状态机
    # ============================================================
    @staticmethod
    def _all_finite(values) -> bool:
        return all(math.isfinite(v) for v in values)

    def _corrected_gyro_from_raw(
        self,
        gyro_raw: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        return (
            gyro_raw[0] - self.gyro_bias_x,
            gyro_raw[1] - self.gyro_bias_y,
            gyro_raw[2] - self.gyro_bias_z,
        )

    def _inputs_fresh(self, now_s: float) -> bool:
        if (
            self.latest_imu_receive_s is None
            or self.latest_encoder_receive_s is None
            or self.latest_imu is None
            or self.latest_encoder is None
        ):
            return False

        if now_s - self.latest_imu_receive_s > IMU_STALE_TIMEOUT_S:
            return False
        if now_s - self.latest_encoder_receive_s > ENCODER_STALE_TIMEOUT_S:
            return False
        return True

    def _is_stationary(self, now_s: float) -> bool:
        """严格按用户给定条件判断静止。

        编码器：
          |Vx|, |Vy|, |Vz| < 0.02 m/s
          |dVx|, |dVy|, |dVz| < 0.02 m/s

        IMU 陀螺仪：
          校正后 |Gx|, |Gy|, |Gz| < 0.02 rad/s
          |dGx|, |dGy|, |dGz| < 0.02 rad/s

        IMU 加速度：
          只检查 |dAx|, |dAy|, |dAz| < 2.00 m/s²。
          不检查加速度绝对值，因为静止时仍包含约 9.8 m/s² 重力。
        """
        if not self._inputs_fresh(now_s):
            return False

        if self.encoder_delta is None or self.gyro_delta is None or self.accel_delta is None:
            return False

        vel = (
            float(self.latest_encoder.vector.x),
            float(self.latest_encoder.vector.y),
            float(self.latest_encoder.vector.z),
        )
        gyro_raw = (
            float(self.latest_imu.angular_velocity.x),
            float(self.latest_imu.angular_velocity.y),
            float(self.latest_imu.angular_velocity.z),
        )
        gyro = self._corrected_gyro_from_raw(gyro_raw)

        encoder_abs_ok = all(abs(v) < VEL_ABS_THRESHOLD for v in vel)
        encoder_delta_ok = all(d < VEL_DELTA_THRESHOLD for d in self.encoder_delta)
        gyro_abs_ok = all(abs(g) < GYRO_ABS_THRESHOLD for g in gyro)
        gyro_delta_ok = all(d < GYRO_DELTA_THRESHOLD for d in self.gyro_delta)
        accel_delta_ok = all(d < ACCEL_DELTA_THRESHOLD for d in self.accel_delta)

        return (
            encoder_abs_ok
            and encoder_delta_ok
            and gyro_abs_ok
            and gyro_delta_ok
            and accel_delta_ok
        )

    def _reset_collection(self) -> None:
        self.collect_start_s = None
        self.collect_sum_x = 0.0
        self.collect_sum_y = 0.0
        self.collect_sum_z = 0.0
        self.collect_count = 0

    def _start_collection(
        self,
        now_s: float,
        gyro_raw: Tuple[float, float, float],
    ) -> None:
        self.calibration_state = self.STATE_COLLECTING
        self._reset_collection()
        self.collect_start_s = now_s
        self._append_calibration_sample(gyro_raw)

        mode_text = "首次" if not self.first_calibration_done else "自动"
        self.get_logger().info(
            f"小车已连续静止 {STILL_REQUIRED_S:.0f}s，进入{mode_text}零飘采集，"
            f"采集时间 {CALIBRATION_COLLECT_S:.0f}s",
            throttle_duration_sec=self.info_time_s,
        )

    def _append_calibration_sample(
        self,
        gyro_raw: Tuple[float, float, float],
    ) -> None:
        self.collect_sum_x += gyro_raw[0]
        self.collect_sum_y += gyro_raw[1]
        self.collect_sum_z += gyro_raw[2]
        self.collect_count += 1

    def _reset_ekf_after_first_calibration(self) -> None:
        """首次 IMU 零飘完成后复位 robot_localization 状态。"""
        if self.ekf_reset_done:
            return

        if not self.ekf_reset_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warning(
                "未找到 EKF set_pose 服务，跳过融合复位",
                throttle_duration_sec=self.info_time_s,
            )
            return

        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = ODOM_FRAME_ID
        request.pose.pose.pose.orientation.w = 1.0
        request.pose.pose.covariance[0] = 1e-9
        request.pose.pose.covariance[7] = 1e-9
        request.pose.pose.covariance[35] = 1e-9
        self.ekf_reset_client.call_async(request)
        self.ekf_reset_done = True
        self.get_logger().info("首次 IMU 校准完成，已请求 EKF 重新初始化")

    def _finish_calibration(self, now_s: float) -> None:
        if self.collect_count <= 0:
            self.get_logger().error(
                "零飘采集样本数为 0，无法完成校准",
                throttle_duration_sec=self.info_time_s,
            )
            self.calibration_state = self.STATE_WAIT_STILL
            self.still_start_s = None
            self._reset_collection()
            return

        # 直接对“原始角速度”求静止平均值，得到新的绝对零飘偏置。
        self.gyro_bias_x = self.collect_sum_x / self.collect_count
        self.gyro_bias_y = self.collect_sum_y / self.collect_count
        self.gyro_bias_z = self.collect_sum_z / self.collect_count

        was_first = not self.first_calibration_done
        self.first_calibration_done = True
        self.last_calibration_done_s = now_s
        if was_first:
            self._reset_ekf_after_first_calibration()
        self.calibration_state = self.STATE_WAIT_STILL

        # 当前此刻仍满足静止，因此从校准完成时重新累计静止时间。
        # 30 s 保护期结束后，如果小车始终未动，可以立即进入下一次 5 s 采集。
        self.still_start_s = now_s

        samples = self.collect_count
        self._reset_collection()

        mode_text = "首次" if was_first else "自动"
        self.get_logger().info(
            f"{mode_text}零飘校准完成: samples={samples}, "
            f"bias_x={self.gyro_bias_x:.8f} rad/s, "
            f"bias_y={self.gyro_bias_y:.8f} rad/s, "
            f"bias_z={self.gyro_bias_z:.8f} rad/s; "
            f"未来 {CALIBRATION_VALID_S:.0f}s 内不再次进入零飘采集",
            throttle_duration_sec=self.info_time_s,
        )

        if was_first:
            self.get_logger().info(
                "\n"
                "==================================================\n"
                "==================================================\n"
                "==========                              ==========\n"
                "==========       首次零飘校准成功       ==========\n"
                "==========                              ==========\n"
                "==================================================\n"
                "=================================================="
            )

    def _update_calibration(
        self,
        now_s: float,
        gyro_raw: Tuple[float, float, float],
    ) -> None:
        stationary = self._is_stationary(now_s)

        # --------------------------------------------------------
        # 状态 1：等待连续静止 5 s
        # --------------------------------------------------------
        if self.calibration_state == self.STATE_WAIT_STILL:
            if not stationary:
                if self.still_start_s is not None:
                    mode_text = "首次" if not self.first_calibration_done else "自动"
                    self.get_logger().warning(
                        f"检测到小车移动，{mode_text}校准静止计时重新开始",
                        throttle_duration_sec=self.info_time_s,
                    )
                self.still_start_s = None
                return

            if self.still_start_s is None:
                self.still_start_s = now_s

            still_elapsed = now_s - self.still_start_s

            # 首次校准完成后才启用自动校准，并执行 30 s 有效/保护期。
            if self.first_calibration_done and self.last_calibration_done_s is not None:
                since_done = now_s - self.last_calibration_done_s
                cooldown_left = CALIBRATION_VALID_S - since_done
                if cooldown_left > 0.0:
                    self.get_logger().info(
                        f"小车静止 {still_elapsed:.1f}s，零飘值仍在有效期，"
                        f"自动校准保护剩余 {cooldown_left:.1f}s",
                        throttle_duration_sec=self.info_time_s,
                    )
                    return

            if still_elapsed < STILL_REQUIRED_S:
                mode_text = "首次" if not self.first_calibration_done else "自动"
                self.get_logger().info(
                    f"等待{mode_text}零飘校准: 已连续静止 {still_elapsed:.1f}/"
                    f"{STILL_REQUIRED_S:.1f}s",
                    throttle_duration_sec=self.info_time_s,
                )
                return

            self._start_collection(now_s, gyro_raw)
            return

        # --------------------------------------------------------
        # 状态 2：零飘采集 5 s
        # --------------------------------------------------------
        if self.calibration_state == self.STATE_COLLECTING:
            if not stationary:
                if not self.first_calibration_done:
                    # 首次校准中途移动：整个流程重新开始，重新等待连续静止 5 s。
                    self.get_logger().warning(
                        "首次零飘采集中检测到移动，本次样本全部丢弃，重新等待静止 5s",
                        throttle_duration_sec=self.info_time_s,
                    )
                else:
                    # 自动校准中途移动：保留原偏置，不修改当前已生效的零飘值。
                    self.get_logger().warning(
                        "自动零飘采集中检测到移动，放弃本次校准并继续使用原零飘值",
                        throttle_duration_sec=self.info_time_s,
                    )

                self.calibration_state = self.STATE_WAIT_STILL
                self.still_start_s = None
                self._reset_collection()
                return

            self._append_calibration_sample(gyro_raw)
            collect_elapsed = now_s - self.collect_start_s

            self.get_logger().info(
                f"零飘采集中: {collect_elapsed:.1f}/{CALIBRATION_COLLECT_S:.1f}s, "
                f"samples={self.collect_count}",
                throttle_duration_sec=self.info_time_s,
            )

            if collect_elapsed >= CALIBRATION_COLLECT_S:
                self._finish_calibration(now_s)

    # ============================================================
    # 4. /imu_raw 发布，100 Hz
    # ============================================================
    def Publish_ImuSensor(self, source_msg: Imu) -> None:
        # 每个输入样本只发布一次，直接继承上游采样时间。
        Imu_Data_Pub = Imu()

        # 保留上游 IMU 采样时间，不使用当前 ROS 时间覆盖。
        # 上游 imu_raw_raw 已把传感器轴旋转到车体 XYZ；这里不再旋转数值，
        # 只在 ekf_bringup 输出时把 IMU 帧统一命名为 imu_link，并由本功能包 robot_tf.launch.py 中的静态 TF 描述其安装位置。
        Imu_Data_Pub.header.stamp = source_msg.header.stamp
        Imu_Data_Pub.header.frame_id = IMU_FRAME_ID

        # /imu_raw 不直接给 orientation。
        Imu_Data_Pub.orientation.w = 1.0
        Imu_Data_Pub.orientation_covariance[0] = -1.0

        # 原始三轴角速度减去已经生效的零飘偏置，单位 rad/s。
        Imu_Data_Pub.angular_velocity.x = (
            source_msg.angular_velocity.x - self.gyro_bias_x
        )
        Imu_Data_Pub.angular_velocity.y = (
            source_msg.angular_velocity.y - self.gyro_bias_y
        )
        Imu_Data_Pub.angular_velocity.z = (
            source_msg.angular_velocity.z - self.gyro_bias_z
        )

        # X/Y 角速度给出极大协方差，Z 轴角速度给出实际使用的较小方差。
        # 在 angular_velocity 三个分量中，ekf.yaml 的 imu0_config 只打开 Z 轴；
        # 同一配置还会融合 Madgwick 提供的 yaw（orientation 绕 Z 轴）。
        # 大协方差本身只是表示低可信度，并不等价于关闭 robot_localization 融合项。
        Imu_Data_Pub.angular_velocity_covariance[0] = COV_DISABLE
        Imu_Data_Pub.angular_velocity_covariance[4] = COV_DISABLE
        Imu_Data_Pub.angular_velocity_covariance[8] = GYRO_Z_COV

        # 加速度保留上游当前三轴测量及重力分量，单位 m/s²。
        # 不对加速度做静止归零，否则会破坏姿态滤波器需要的重力方向信息。
        Imu_Data_Pub.linear_acceleration.x = source_msg.linear_acceleration.x
        Imu_Data_Pub.linear_acceleration.y = source_msg.linear_acceleration.y
        Imu_Data_Pub.linear_acceleration.z = source_msg.linear_acceleration.z

        # 与用户给出的 Publish_ImuSensor() 示例保持一致，供姿态滤波器使用。
        # 当前 robot_localization 的 imu0_config 关闭 ax/ay/az 线加速度；这些数据保留给 Madgwick/其它节点使用。
        Imu_Data_Pub.linear_acceleration_covariance[0] = ACCEL_COV
        Imu_Data_Pub.linear_acceleration_covariance[4] = ACCEL_COV
        Imu_Data_Pub.linear_acceleration_covariance[8] = ACCEL_COV

        self.imu_publisher.publish(Imu_Data_Pub)

        self.get_logger().info(
            "IMU: "
            f"Ax={Imu_Data_Pub.linear_acceleration.x:.4f}, "
            f"Ay={Imu_Data_Pub.linear_acceleration.y:.4f}, "
            f"Az={Imu_Data_Pub.linear_acceleration.z:.4f} m/s² | "
            f"Gx={Imu_Data_Pub.angular_velocity.x:.5f}, "
            f"Gy={Imu_Data_Pub.angular_velocity.y:.5f}, "
            f"Gz={Imu_Data_Pub.angular_velocity.z:.5f} rad/s | "
            f"bias=({self.gyro_bias_x:.5f}, {self.gyro_bias_y:.5f}, {self.gyro_bias_z:.5f})",
            throttle_duration_sec=self.info_time_s,
        )

    # ============================================================
    # 5. /odom 发布，50 Hz
    # ============================================================
    def Publish_Odom(self, source_msg: Vector3Stamped) -> None:
        # 每个编码器样本只发布一次，直接继承编码器 timestamp。
        odom = Odometry()

        odom.header.stamp = source_msg.header.stamp
        odom.header.frame_id = ODOM_FRAME_ID
        odom.child_frame_id = BASE_FRAME_ID

        # 不向 EKF 提供编码器位姿，只保留合法空位姿。
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # pose 不由编码器节点提供，因此所有 pose 对角协方差都设为极大值。
        # ekf.yaml 同时把 odom pose 对应配置项全部关闭。
        odom.pose.covariance[0] = COV_DISABLE
        odom.pose.covariance[7] = COV_DISABLE
        odom.pose.covariance[14] = COV_DISABLE
        odom.pose.covariance[21] = COV_DISABLE
        odom.pose.covariance[28] = COV_DISABLE
        odom.pose.covariance[35] = COV_DISABLE

        # 编码器只给 EKF Vx、Vy；上游 AKM 节点的 Vy/Vz 当前均为 0。
        odom.twist.twist.linear.x = float(source_msg.vector.x)
        odom.twist.twist.linear.y = float(source_msg.vector.y)
        odom.twist.twist.linear.z = 0.0

        # 编码器角速度不送入 EKF，yaw rate 统一使用 IMU Gz。
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # vx、vy 给出正常测量方差，其它 twist 分量给出极大方差。
        # 最终只融合 vx、vy 由 ekf.yaml 的 odom0_config 明确决定。
        odom.twist.covariance[0] = VX_COV
        odom.twist.covariance[7] = VY_COV
        odom.twist.covariance[14] = COV_DISABLE
        odom.twist.covariance[21] = COV_DISABLE
        odom.twist.covariance[28] = COV_DISABLE
        odom.twist.covariance[35] = COV_DISABLE

        self.odom_publisher.publish(odom)

        self.get_logger().info(
            f"ODOM: Vx={odom.twist.twist.linear.x:.4f} m/s, "
            f"Vy={odom.twist.twist.linear.y:.4f} m/s, "
            f"Vz={odom.twist.twist.linear.z:.4f} m/s",
            throttle_duration_sec=self.info_time_s,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
