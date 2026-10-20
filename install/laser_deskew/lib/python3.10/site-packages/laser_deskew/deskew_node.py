#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""2D LaserScan motion deskew for a rotating LiDAR.

The Leishen X10 driver in this workspace stamps each LaserScan with the sweep
end time and sets time_increment to zero.  This node reconstructs each ray's
acquisition time from its angular bin, interpolates /odom_combined, transforms
all rays to the LiDAR pose at the sweep end, and publishes /scan_deskewed.

Assumptions for this vehicle:
- planar motion (x, y, yaw),
- /odom_combined pose is odom -> akm_rear_axle_footprint,
- the LiDAR is rigidly mounted to the base,
- X10/M10 points increase in raw azimuth from 0 to 360 degrees, while the
  driver's LaserScan conversion uses atan2(-y, x). Therefore per-ray time must
  be reconstructed from azimuth phase, not directly from LaserScan array index.
"""

from __future__ import annotations

import bisect
import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass
class Pose2D:
    """二维位姿及其时间戳。

    t 使用秒表示；x、y 位于 odom 坐标系；yaw 为绕 Z 轴的航向角（弧度）。
    """

    t: float
    x: float
    y: float
    yaw: float


def stamp_to_sec(stamp) -> float:
    """将 ROS 2 builtin_interfaces/Time 转成浮点秒。"""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def sec_to_stamp(value: float):
    """将浮点秒转换成 ROS 2 Time 消息，并处理纳秒进位。"""
    sec = math.floor(value)
    nanosec = int(round((value - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    from builtin_interfaces.msg import Time

    msg = Time()
    msg.sec = int(sec)
    msg.nanosec = int(max(0, nanosec))
    return msg


def wrap_angle(a: float) -> float:
    """把角度归一化到 [-pi, pi]，避免跨越 ±pi 时插值走远路。"""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q) -> float:
    """从四元数中提取 yaw；本节点只处理二维平面运动。"""
    # 标准四元数 -> yaw 公式；对于二维 EKF/里程计输出已经足够。
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LaserDeskewNode(Node):
    """利用融合里程计对旋转式 2D LaserScan 做运动去畸变。

    功能：
        将一圈扫描中在不同时刻采到的激光点，统一补偿到同一个参考时刻，
        减少车辆运动导致的墙体弯曲、重影和扫描错位。

    使用前最重要的配置：
        1. input_scan / input_odom / output_scan：确认话题名正确；
        2. laser_x / laser_y / laser_yaw：填写真实雷达安装外参；
        3. scan_duration：接近 1 / 雷达扫描频率；
        4. stamp_is_end：根据雷达驱动时间戳定义设置；
        5. raw_azimuth_increases：根据原始点云扫描方向设置。

    具体调参说明见 config/deskew.yaml。
    """

    def __init__(self) -> None:
        super().__init__("laser_deskew")

        # =========================================================
        # ROS 参数：输入 / 输出话题
        # =========================================================
        # input_scan
        #   功能：原始 LaserScan 的输入话题。
        #   怎么设置：填雷达驱动实际发布的 LaserScan 话题，通常为 /scan。
        self.declare_parameter("input_scan", "/scan")

        # input_odom
        #   功能：运动补偿使用的里程计话题，用于插值每束激光采样时刻的车辆位姿。
        #   怎么设置：优先选择高频、连续、已融合 IMU/轮速的 odom；当前使用 /odom_combined。
        self.declare_parameter("input_odom", "/odom_combined")

        # output_scan
        #   功能：去畸变后的 LaserScan 输出话题。
        #   怎么设置：建议与原始 /scan 分开，例如 /scan_deskewed，便于 RViz 对比和下游切换。
        self.declare_parameter("output_scan", "/scan_deskewed")

        # =========================================================
        # ROS 参数：雷达相对 AKM 运动学参考点的二维安装外参
        # =========================================================
        # laser_x
        #   功能：雷达相对 akm_rear_axle_footprint 的前后偏移，单位 m。
        #   怎么设置：akm_rear_axle_footprint 到 base_footprint 为 0.17 m，
        #             base_link 到 laser_link 的 x 偏移为 0.08 m，因此平面 x = 0.25 m。
        self.declare_parameter("laser_x", 0.25)

        # laser_y
        #   功能：雷达相对 akm_rear_axle_footprint 的左右偏移，单位 m。
        #   怎么设置：直接取 TF/URDF 的 y；按 ROS 坐标约定通常左为正、右为负。
        self.declare_parameter("laser_y", 0.0)

        # laser_yaw
        #   功能：雷达相对 akm_rear_axle_footprint 正前方的偏航角，单位 rad。
        #   怎么设置：从 TF/URDF 读取；朝向完全一致填 0，左转 90° 约 +1.5708。
        self.declare_parameter("laser_yaw", 0.0)

        # =========================================================
        # ROS 参数：单圈扫描周期
        # =========================================================
        # scan_duration
        #   功能：第一帧或时间戳异常时使用的默认单圈扫描时间，单位 s。
        #   怎么设置：约等于 1 / 雷达频率，例如 10 Hz -> 0.10 s；可用 ros2 topic hz /scan 测量。
        self.declare_parameter("scan_duration", 0.10)

        # min_scan_duration
        #   功能：自动估计扫描周期时允许的最小值，用来过滤异常小的时间间隔。
        #   怎么设置：应小于正常 scan_duration；10 Hz 雷达设置 0.03 s 通常足够宽松。
        self.declare_parameter("min_scan_duration", 0.03)

        # max_scan_duration
        #   功能：自动估计扫描周期时允许的最大值，用来过滤卡顿/丢帧造成的异常大间隔。
        #   怎么设置：应大于正常 scan_duration；10 Hz 雷达可从 0.2~0.3 s 左右开始。
        self.declare_parameter("max_scan_duration", 0.30)

        # stamp_is_end
        #   功能：说明 LaserScan.header.stamp 是一圈扫描的结束时刻还是开始时刻。
        #   怎么设置：结束时刻填 true，开始时刻填 false；必须根据驱动源码/文档确认。
        #   当前 X10 驱动把 sweep_end_time 写入 header.stamp，所以设置 true。
        self.declare_parameter("stamp_is_end", True)

        # raw_azimuth_increases
        #   功能：说明原始方位角随采样时间是递增还是递减，决定每束激光的时间重建顺序。
        #   怎么设置：0°->360° 为 true，360°->0° 为 false；可运行 check_scan_direction.py 自动判断。
        self.declare_parameter("raw_azimuth_increases", True)

        # =========================================================
        # ROS 参数：里程计缓存和时间容错
        # =========================================================
        # odom_history_seconds
        #   功能：保存最近多少秒的里程计历史，供扫描期间的位姿插值使用。
        #   怎么设置：必须大于 scan_duration；一般 1~3 s 足够，当前 2.0 s。
        self.declare_parameter("odom_history_seconds", 2.0)

        # max_extrapolation
        #   功能：允许查询时间稍微越过 odom 缓存边界的最大容差，单位 s。
        #   怎么设置：尽量小，只覆盖正常消息抖动；通常可从 0.01~0.05 s 范围尝试。
        #   过大会在快速运动时降低精度；若必须很大，应先排查时间同步问题。
        self.declare_parameter("max_extrapolation", 0.03)

        # pending_scan_limit
        #   功能：限制等待 odom 数据的 LaserScan 最多缓存多少帧。
        #   怎么设置：只需覆盖正常 odom 延迟；频繁溢出应先检查 odom 是否掉线/严重延迟。
        self.declare_parameter("pending_scan_limit", 5)

        # 读取参数并转换为后续运算需要的 Python 类型。
        self.input_scan = str(self.get_parameter("input_scan").value)
        self.input_odom = str(self.get_parameter("input_odom").value)
        self.output_scan = str(self.get_parameter("output_scan").value)
        self.laser_x = float(self.get_parameter("laser_x").value)
        self.laser_y = float(self.get_parameter("laser_y").value)
        self.laser_yaw = float(self.get_parameter("laser_yaw").value)
        self.fallback_duration = float(self.get_parameter("scan_duration").value)
        self.min_duration = float(self.get_parameter("min_scan_duration").value)
        self.max_duration = float(self.get_parameter("max_scan_duration").value)
        self.stamp_is_end = bool(self.get_parameter("stamp_is_end").value)
        self.raw_azimuth_increases = bool(self.get_parameter("raw_azimuth_increases").value)
        self.history_seconds = float(self.get_parameter("odom_history_seconds").value)
        self.max_extrapolation = float(self.get_parameter("max_extrapolation").value)
        self.pending_limit = int(self.get_parameter("pending_scan_limit").value)

        # odom：按时间递增保存最近一段里程计位姿，用于时间插值。
        # pending：缓存暂时等不到足够里程计覆盖的 LaserScan。
        self.odom: Deque[Pose2D] = deque()
        self.pending: Deque[Tuple[LaserScan, float]] = deque()
        self.last_scan_stamp: Optional[float] = None
        self.filtered_duration: Optional[float] = None

        # 发布去畸变后的扫描；分别订阅里程计和原始扫描。
        # 5 ms 定时器用于在新里程计到达后继续尝试处理 pending 队列。
        self.pub = self.create_publisher(LaserScan, self.output_scan, 10)
        self.create_subscription(Odometry, self.input_odom, self.odom_callback, 100)
        self.create_subscription(LaserScan, self.input_scan, self.scan_callback, 10)
        self.create_timer(0.005, self.process_pending)

        self.get_logger().info(
            f"deskew: {self.input_scan} + {self.input_odom} -> {self.output_scan}; "
            f"laser=(x={self.laser_x:.3f}, y={self.laser_y:.3f}, yaw={self.laser_yaw:.3f}), "
            f"stamp_is_end={self.stamp_is_end}, raw_azimuth_increases={self.raw_azimuth_increases}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        """接收并缓存里程计位姿。

        功能：把 /odom_combined 中的位姿转换成二维 x/y/yaw，并按时间保存，
        后续用于查询某一束激光采样时刻的机器人位姿。

        参数：
            msg: Odometry 消息。其 header.stamp 必须与 LaserScan 使用同一时间基准，
                 pose 应对应 odom 坐标系下的底盘位姿。
        """
        t = stamp_to_sec(msg.header.stamp)
        if t <= 0.0:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        pose = Pose2D(t=t, x=float(p.x), y=float(p.y), yaw=yaw_from_quaternion(q))

        # robot_localization 的输出时间应当单调递增。
        # 极少数重复或乱序数据直接丢弃，避免破坏后续二分查找/插值。
        if self.odom and t <= self.odom[-1].t:
            return
        self.odom.append(pose)

        # 只保留 history_seconds 指定的时间窗口；至少保留两个点，
        # 这样边界附近仍有机会进行线性插值。
        cutoff = t - self.history_seconds
        while len(self.odom) > 2 and self.odom[1].t < cutoff:
            self.odom.popleft()

    def scan_callback(self, msg: LaserScan) -> None:
        """接收原始 LaserScan，并准备进行去畸变。

        功能：估计当前扫描周期，把扫描放入等待队列；只有当所需时间范围内的
        odom 位姿已经可用时，才真正执行去畸变。

        参数：
            msg: 原始 LaserScan。话题由 input_scan 参数指定。
        """
        stamp = stamp_to_sec(msg.header.stamp)
        if stamp <= 0.0 or len(msg.ranges) < 2:
            return

        # 先得到这一圈扫描的持续时间，随后等待完整里程计时间覆盖。
        duration = self._estimate_duration(msg, stamp)
        self.pending.append((msg, duration))
        while len(self.pending) > self.pending_limit:
            self.pending.popleft()
            self.get_logger().warning(
                "deskew pending queue overflow; dropping oldest scan. Check odom/scan timestamps.",
                throttle_duration_sec=2.0,
            )
        self.process_pending()

    def _estimate_duration(self, msg: LaserScan, stamp: float) -> float:
        """估计当前一圈激光扫描持续时间。

        功能：优先使用 LaserScan.scan_time；若不可用，则使用相邻消息时间戳差；
        再用低通滤波抑制调度抖动。异常值会退回 scan_duration 参数。

        参数：
            msg: 当前 LaserScan。
            stamp: 当前 LaserScan.header.stamp 转换后的秒数。

        返回：
            估计并平滑后的扫描周期，单位 s。
        """
        # 如果驱动提供了可信的 scan_time，优先使用元数据。
        if math.isfinite(msg.scan_time) and self.min_duration <= msg.scan_time <= self.max_duration:
            raw = float(msg.scan_time)
        elif self.last_scan_stamp is not None:
            raw = stamp - self.last_scan_stamp
            if not (self.min_duration <= raw <= self.max_duration):
                raw = self.fallback_duration
        else:
            raw = self.fallback_duration
        self.last_scan_stamp = stamp

        # 对扫描周期做一阶低通，避免调度/时间戳抖动被转换成几何畸变。
        if self.filtered_duration is None:
            self.filtered_duration = raw
        else:
            self.filtered_duration = 0.85 * self.filtered_duration + 0.15 * raw
        return self.filtered_duration

    def process_pending(self) -> None:
        """处理等待队列中已经具备足够 odom 数据的扫描。

        功能：根据 stamp_is_end 和扫描周期计算本圈的起止时间；如果历史里程计
        已覆盖整个扫描时间范围，就执行去畸变；否则继续等待或在确认无法处理时丢帧。
        """
        while self.pending:
            msg, duration = self.pending[0]
            end_stamp = stamp_to_sec(msg.header.stamp)
            if self.stamp_is_end:
                t_start = end_stamp - duration
                t_ref = end_stamp
            else:
                t_start = end_stamp
                t_ref = end_stamp + duration

            # 去畸变必须同时知道整圈扫描期间以及参考时刻的机器人位姿。
            if not self._time_available(t_start) or not self._time_available(t_ref):
                # 如果只是里程计还没追上扫描时间，先保留该帧，等待后续 odom。
                if not self.odom or self.odom[-1].t < t_ref - self.max_extrapolation:
                    return
                # 若里程计已经越过参考时刻仍无法覆盖，说明时间戳不兼容或扫描过旧，丢弃。
                self.pending.popleft()
                self.get_logger().warning(
                    f"cannot bracket scan [{t_start:.6f}, {t_ref:.6f}] with odom history; dropping",
                    throttle_duration_sec=2.0,
                )
                continue

            self.pending.popleft()
            out = self._deskew(msg, duration, t_start, t_ref)
            if out is not None:
                self.pub.publish(out)

    def _time_available(self, t: float) -> bool:
        """判断指定时刻是否有可用里程计。

        参数：
            t: 需要查询的时间，单位 s。

        返回：
            True 表示 t 位于 odom 历史范围内，或只超出不超过 max_extrapolation；
            False 表示当前无法可靠得到该时刻位姿。
        """
        if not self.odom:
            return False
        return (self.odom[0].t - self.max_extrapolation) <= t <= (self.odom[-1].t + self.max_extrapolation)

    def _interp_pose(self, t: float) -> Optional[Pose2D]:
        """在里程计缓存中插值得到时刻 t 的二维位姿。

        x、y 使用线性插值；yaw 先做角度环绕处理再线性插值，
        从而正确处理 +pi/-pi 附近的跨界情况。
        """
        if not self.odom:
            return None
        # 二分查找 t 两侧的里程计样本。
        times = [p.t for p in self.odom]
        idx = bisect.bisect_left(times, t)

        # 缓存边界允许 max_extrapolation 范围内的“保持值”外推；
        # 超过容差则认为没有可靠位姿。
        if idx == 0:
            if abs(times[0] - t) <= self.max_extrapolation:
                p = self.odom[0]
                return Pose2D(t, p.x, p.y, p.yaw)
            return None
        if idx >= len(self.odom):
            if abs(t - times[-1]) <= self.max_extrapolation:
                p = self.odom[-1]
                return Pose2D(t, p.x, p.y, p.yaw)
            return None

        a = self.odom[idx - 1]
        b = self.odom[idx]
        dt = b.t - a.t
        if dt <= 1e-9:
            return Pose2D(t, a.x, a.y, a.yaw)
        u = (t - a.t) / dt
        dyaw = wrap_angle(b.yaw - a.yaw)
        return Pose2D(
            t=t,
            x=a.x + u * (b.x - a.x),
            y=a.y + u * (b.y - a.y),
            yaw=wrap_angle(a.yaw + u * dyaw),
        )

    def _laser_pose_in_odom(self, base: Pose2D) -> Pose2D:
        """由底盘位姿计算同一时刻的雷达位姿。

        功能：把 laser_x / laser_y / laser_yaw 外参叠加到 base 位姿上，得到
        odom 坐标系中的雷达 x/y/yaw。

        参数：
            base: 某一时刻底盘在 odom 中的二维位姿。

        返回：
            同一时刻雷达在 odom 中的二维位姿。
        """
        c = math.cos(base.yaw)
        s = math.sin(base.yaw)
        x = base.x + c * self.laser_x - s * self.laser_y
        y = base.y + s * self.laser_x + c * self.laser_y
        return Pose2D(base.t, x, y, wrap_angle(base.yaw + self.laser_yaw))

    def _deskew(
        self,
        scan: LaserScan,
        duration: float,
        t_start: float,
        t_ref: float,
    ) -> Optional[LaserScan]:
        """把一帧扫描中的所有激光点统一变换到参考时刻的雷达坐标系。

        核心步骤：
        1. 根据每个角度 bin 恢复该激光束在整圈扫描中的采样时刻；
        2. 插值得到该时刻的雷达位姿；
        3. 激光点从“采样时刻雷达系”变换到 odom；
        4. 再从 odom 变换到“参考时刻雷达系”；
        5. 重新投影到 LaserScan 的角度 bin。
        """
        # 参考位姿通常取本圈扫描结束时刻。最终所有点都表达在这个雷达坐标系下。
        ref_base = self._interp_pose(t_ref)
        if ref_base is None:
            return None
        ref = self._laser_pose_in_odom(ref_base)
        cr = math.cos(ref.yaw)
        sr = math.sin(ref.yaw)

        n = len(scan.ranges)
        # 输出仍保持与输入相同的角度分辨率和数组长度。
        # 某些 bin 若没有有效点，保留为 +inf，符合 LaserScan 常见表示方式。
        ranges = [math.inf] * n
        intensities = [0.0] * n if len(scan.intensities) == n else []

        valid_count = 0
        for i, r_value in enumerate(scan.ranges):
            r = float(r_value)
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            # 驱动内部先使用原始方位角：x=r*cos(raw_az), y=r*sin(raw_az)，
            # 随后生成 LaserScan 时使用 angle=atan2(-y, x)=-raw_az（包裹到 [-pi, pi]）。
            # 因此不能简单按 ranges 数组下标推断采样先后，而要反推出 raw_az。
            # 一圈扫描在原始方位角 360° -> 0° 回绕处开始，所以采样相位为 raw_az/(2*pi)。
            raw_az = (-angle) % (2.0 * math.pi)
            if self.raw_azimuth_increases:
                temporal_fraction = raw_az / (2.0 * math.pi)
            else:
                temporal_fraction = ((2.0 * math.pi - raw_az) % (2.0 * math.pi)) / (2.0 * math.pi)
            # temporal_fraction 属于 [0, 1)，据此恢复当前激光束的真实采样时刻。
            t_ray = t_start + temporal_fraction * duration
            ray_base = self._interp_pose(t_ray)
            if ray_base is None:
                continue
            ray_pose = self._laser_pose_in_odom(ray_base)

            # 输入 LaserScan 极坐标 -> 当前采样时刻的雷达笛卡尔坐标。
            px_l = r * math.cos(angle)
            py_l = r * math.sin(angle)

            # 第一次坐标变换：采样时刻 laser -> odom。
            ca = math.cos(ray_pose.yaw)
            sa = math.sin(ray_pose.yaw)
            px_o = ray_pose.x + ca * px_l - sa * py_l
            py_o = ray_pose.y + sa * px_l + ca * py_l

            # 第二次坐标变换：odom -> 参考时刻 laser。
            # 这里等价于应用参考雷达位姿的逆变换。
            dx = px_o - ref.x
            dy = py_o - ref.y
            px_r = cr * dx + sr * dy
            py_r = -sr * dx + cr * dy

            # 把校正后的笛卡尔坐标重新投影回 LaserScan 的极坐标/bin。
            rr = math.hypot(px_r, py_r)
            aa = math.atan2(py_r, px_r)
            j = int(round((aa - scan.angle_min) / scan.angle_increment))
            if 0 <= j < n and scan.range_min <= rr <= scan.range_max:
                # 多个原始点可能落入同一个输出 bin；保留距离更近的点，
                # 避免后方障碍覆盖前方障碍。
                if rr < ranges[j]:
                    ranges[j] = rr
                    if intensities:
                        intensities[j] = float(scan.intensities[i])
                valid_count += 1

        if valid_count == 0:
            self.get_logger().warning("deskew produced no valid rays", throttle_duration_sec=2.0)
            return None

        out = LaserScan()
        out.header = scan.header
        out.header.stamp = sec_to_stamp(t_ref)
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        # 去畸变后的所有激光束都代表同一参考位姿，因此不再有束间时间差。
        out.time_increment = 0.0
        out.scan_time = float(duration)
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = ranges
        out.intensities = intensities
        return out


def main(args=None) -> None:
    """启动 laser_deskew ROS 2 节点。

    参数：
        args: 可选 ROS 2 命令行参数，通常保持默认 None 即可。
    """
    rclpy.init(args=args)
    node = LaserDeskewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
