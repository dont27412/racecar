#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS2 编码器节点：用单圈位置差判向，累计带方向的位移。

0x0003 的符号在所附说明书中有两种描述，因此速度只取其大小，
正负方向由 0x0000 的相邻位置差（含单圈回绕修正）决定。
要求两次有效位置读取之间的实际转动量小于半圈；超过后仅靠单圈
位置无法唯一恢复方向或漏掉的整圈。这里的 distance 是净位移，
不是正反转都增加的总路程；第一帧位置作为位移起点。

用于 EKF 的 header.stamp 采用主机 SYSTEM_TIME，并取速度寄存器对应 20 ms
测量窗的中心时刻；不会再把随后读取位置寄存器的 Modbus 延迟算进速度时间。
"""

import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
import serial


class EncoderRawNode(Node):
    def __init__(self):
        super().__init__("encoder_raw")

        # 编码器/机械参数
        self.wheel_circumference = 0.213       # 编码器轮子周长，单位：m
        self.encoder_resolution = 1024         # 编码器一圈计数
        self.encoder_speed_sample_ms = 20      # 编码器内部角速度采样时间，单位：ms，对应寄存器0x000A
        self.direction = 1                     # 软件方向修正：1 正方向，-1 反方向
        self.sample_time = 0.02                # ROS2 查询周期，单位：s，20 ms = 50 Hz
        self.info_time_s = 1.0                 # INFO/WARN/ERROR 日志节流间隔，单位：s
        # 与 IMU 驱动使用完全相同的主机 SYSTEM_TIME 作为 ROS header.stamp 基准。
        # 不用 timer 回调结束时刻打戳，因为那是“发布时刻”，不是“速度测量时刻”。
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.last_stamp_ns = None

        # 启动配置参数：提高连续写配置寄存器时的稳定性
        self.config_write_retries = 3           # 配置寄存器最多尝试次数
        self.config_retry_delay_s = 0.20        # 写失败后的重试间隔，单位：s
        self.mode_settle_time_s = 0.50          # 写 0x0006 后等待设备完成模式切换/保存
        self.speed_settle_time_s = 0.50         # 写 0x000A 后等待设备完成角速度采样周期切换
        self.modbus_gap_s = 0.005               # Modbus RTU 帧之间留一点空闲时间

        # RS485 参数
        self.port = "/dev/encoder"
        self.baudrate = 57600
        self.encoder_id = 1

        # 状态量（全部使用 SI 单位）
        self.angular_velocity = 0.0            # 角速度，rad/s
        self.acceleration = 0.0                # 线加速度，m/s^2
        self.angle = 0.0                       # 角度，rad
        self.speed = 0.0                       # 线速度，m/s
        self.distance = 0.0                    # 带方向累计位移，m；反转时减小
        self.last_speed = None                 # 上一次有效线速度，m/s；None 表示尚无有效样本
        self.last_time = None                  # 上一次有效样本的 monotonic 时间，s
        self.last_position = None              # 上一次有效单圈位置；第一帧只建立基准
        self.last_direction = 0                # 位置增量方向；0 表示方向尚未确定

        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.05,
            )
            self.get_logger().info(
                "编码器串口连接成功",
                throttle_duration_sec=self.info_time_s,
            )
        except Exception as e:
            self.get_logger().error(
                f"编码器串口连接失败: {e}",
                throttle_duration_sec=self.info_time_s,
            )
            raise

        # 串口打开后稍等设备稳定
        time.sleep(0.1)

        # 强制编码器进入查询模式，避免自动回传数据与 0x03 查询响应混在一起。
        # 0x0006：编码器模式；0x0000 = 查询模式，功能码 0x06。
        if self.write_register_retry(0x0006, 0x0000):
            self.get_logger().info(
                "编码器已设置为查询模式",
                throttle_duration_sec=self.info_time_s,
            )
        else:
            raise RuntimeError("无法将编码器设置为查询模式（0x0006=0x0000）")

        # 关键修改：0x0006 写入后不要只等 20 ms。
        # 该参数立即生效且掉电记忆，设备可能需要时间完成模式切换/配置保存。
        # 等待 500 ms 后再写 0x000A，可以避免第二条配置命令紧跟着发送而无响应。
        time.sleep(self.mode_settle_time_s)
        self.ser.reset_input_buffer()

        # 将编码器内部角速度采样时间设置为 20 ms。
        # 0x000A：编码器角速度采样时间，单位 ms，功能码 0x06。
        # 后续速度换算依赖这个值；最多重试 3 次，全部失败才退出。
        if self.write_register_retry(0x000A, self.encoder_speed_sample_ms):
            self.get_logger().info(
                f"编码器角速度采样时间已设置为 {self.encoder_speed_sample_ms} ms",
                throttle_duration_sec=self.info_time_s,
            )
        else:
            raise RuntimeError(
                f"无法设置编码器角速度采样时间（0x000A={self.encoder_speed_sample_ms} ms）"
            )

        # 0x000A 修改后等待编码器完成角速度采样周期切换，
        # 避免启动后的第一次 0x03 查询恰好落在设备内部切换阶段。
        time.sleep(self.speed_settle_time_s)
        self.ser.reset_input_buffer()

        # 发布 Vx、Vy、Vz + ROS 时间戳
        # geometry_msgs/Vector3Stamped:
        #   header.stamp -> 时间戳
        #   vector.x     -> Vx，m/s
        #   vector.y     -> Vy，m/s
        #   vector.z     -> Vz，m/s
        self.speed_pub = self.create_publisher(
            Vector3Stamped,
            "/encoder_raw",
            10,
        )

        self.timer = self.create_timer(
            self.sample_time,
            self.timer_callback,
        )

    # CRC16 / Modbus RTU
    def crc16(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    # Modbus 异常码说明
    def modbus_exception_text(self, code):
        return {
            0x01: "非法功能",
            0x02: "非法数据地址",
            0x03: "非法数据值",
            0x04: "从站设备故障",
        }.get(code, "未知异常")

    def check_crc(self, data):
        if len(data) < 4:
            return False
        crc_recv = data[-2] | (data[-1] << 8)
        crc_calc = self.crc16(data[:-2])
        return crc_recv == crc_calc

    def bytes_to_hex(self, data):
        """把串口字节格式化为十六进制字符串，便于排查 RS485/Modbus 通讯。"""
        return " ".join(f"{b:02X}" for b in data)

    def write_register_retry(self, address, value, retries=None):
        """写配置寄存器，失败时等待后重试。"""
        if retries is None:
            retries = self.config_write_retries

        for attempt in range(1, retries + 1):
            if self.write_register(address, value):
                return True

            if attempt < retries:
                self.get_logger().warning(
                    f"写寄存器失败，准备重试 {attempt + 1}/{retries}: "
                    f"address=0x{address:04X}, value=0x{value:04X}"
                )
                time.sleep(self.config_retry_delay_s)
                self.ser.reset_input_buffer()

        return False

    # MODBUS 写单个寄存器，功能码 0x06
    def write_register(self, address, value):
        frame = bytearray([
            self.encoder_id,
            0x06,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ])

        crc = self.crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        # 写之前清空旧数据。启动时若编码器仍处于自动回传模式，
        # 下面会在接收流中寻找 0x06 的正确回显帧，而不是假设前 8 字节就是响应。
        self.ser.reset_input_buffer()
        time.sleep(self.modbus_gap_s)

        self.get_logger().info(
            f"TX 0x06: {self.bytes_to_hex(frame)}"
        )

        self.ser.write(frame)
        self.ser.flush()

        # 配置写入时给从站更宽松的响应窗口。
        deadline = time.monotonic() + max(0.30, self.ser.timeout * 6)
        rx = bytearray()
        expected = bytes(frame)

        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting > 0 else 1)
            if not chunk:
                continue
            rx.extend(chunk)

            # 0x06 正常响应原样回显请求帧。
            if expected in rx:
                self.get_logger().info(
                    f"RX 0x06: {self.bytes_to_hex(rx)}"
                )
                return True

            # Modbus 异常响应固定为：ADR, FUNC|0x80, EX_CODE, CRC_L, CRC_H。
            for i in range(max(0, len(rx) - 16), max(0, len(rx) - 4)):
                candidate = rx[i:i + 5]
                if (
                    len(candidate) == 5
                    and candidate[0] == self.encoder_id
                    and candidate[1] == 0x86
                    and self.check_crc(candidate)
                ):
                    code = candidate[2]
                    self.get_logger().error(
                        f"写寄存器 Modbus 异常: code=0x{code:02X} "
                        f"({self.modbus_exception_text(code)})",
                        throttle_duration_sec=self.info_time_s,
                    )
                    return False

            # 防止异常串口数据无限增长。
            if len(rx) > 256:
                del rx[:-64]

        rx_hex = self.bytes_to_hex(rx) if rx else "<empty>"
        self.get_logger().warning(
            f"写寄存器未收到有效响应: address=0x{address:04X}, "
            f"value=0x{value:04X}, received={len(rx)} bytes, rx={rx_hex}"
        )
        return False

    # MODBUS 读取保持寄存器，功能码 0x03
    # num=1 时返回 int；num>1 时返回寄存器值列表。
    def read_register(self, address, num=1):
        if not isinstance(num, int) or num <= 0 or num > 125:
            raise ValueError("num 必须是 1~125 的整数")

        frame = bytearray([
            self.encoder_id,
            0x03,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (num >> 8) & 0xFF,
            num & 0xFF,
        ])

        crc = self.crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        # 查询模式下每次事务前清空残留字节，避免上一帧残片污染本次响应。
        self.ser.reset_input_buffer()
        time.sleep(self.modbus_gap_s)
        self.ser.write(frame)
        self.ser.flush()

        # 先读 ADR、功能码、byte count/exception code。
        header = self.ser.read(3)
        if len(header) != 3:
            self.get_logger().warning(
                f"读寄存器响应头长度错误: {len(header)}",
                throttle_duration_sec=self.info_time_s,
            )
            return None

        if header[0] != self.encoder_id:
            self.get_logger().warning(
                f"编码器 ID 不匹配: recv={header[0]}, expected={self.encoder_id}",
                throttle_duration_sec=self.info_time_s,
            )
            return None

        # 0x03 的异常功能码为 0x83；此时第 3 字节就是异常码。
        if header[1] == 0x83:
            tail = self.ser.read(2)
            data = header + tail
            if len(tail) != 2:
                self.get_logger().warning(
                    "Modbus 异常响应长度不完整",
                    throttle_duration_sec=self.info_time_s,
                )
                return None
            if not self.check_crc(data):
                self.get_logger().warning(
                    "Modbus 异常响应 CRC 校验失败",
                    throttle_duration_sec=self.info_time_s,
                )
                return None

            code = header[2]
            self.get_logger().error(
                f"读寄存器 Modbus 异常: code=0x{code:02X} "
                f"({self.modbus_exception_text(code)})",
                throttle_duration_sec=self.info_time_s,
            )
            return None

        if header[1] != 0x03:
            self.get_logger().warning(
                f"Modbus 功能码错误: 0x{header[1]:02X}",
                throttle_duration_sec=self.info_time_s,
            )
            return None

        byte_count = header[2]
        expected_byte_count = num * 2
        if byte_count != expected_byte_count:
            self.get_logger().warning(
                f"读寄存器 byte count 错误: recv={byte_count}, "
                f"expected={expected_byte_count}",
                throttle_duration_sec=self.info_time_s,
            )
            # byte_count 已经不符合请求，不按异常值继续读取大量数据。
            return None

        # 数据区 + CRC 两字节。正常总长度 = 3 + byte_count + 2。
        tail = self.ser.read(byte_count + 2)
        if len(tail) != byte_count + 2:
            self.get_logger().warning(
                f"读寄存器响应长度错误: recv={3 + len(tail)}, "
                f"expected={5 + byte_count}",
                throttle_duration_sec=self.info_time_s,
            )
            return None

        data = header + tail
        if not self.check_crc(data):
            self.get_logger().warning(
                "读寄存器 CRC 校验失败",
                throttle_duration_sec=self.info_time_s,
            )
            return None

        registers = [
            (data[3 + 2 * i] << 8) | data[4 + 2 * i]
            for i in range(num)
        ]

        if num == 1:
            return registers[0]
        return registers

    # 读取角速度原始值 0x0003，保留原有的 int16 解码。
    # 其实际符号供日志排查，运动方向统一由位置差确定。
    def read_speed_value(self):
        value = self.read_register(0x0003)
        if value is None:
            return None

        if value >= 32768:
            value -= 65536

        return value

    # 读取位置原始值 0x0000
    def read_position(self):
        return self.read_register(0x0000)

    def get_position_delta(self, position):
        """返回回绕修正后的计数增量；无基准或恰好半圈时返回 None。

        以 1024 计数/圈为例：1020 -> 4 为 +8，4 -> 1020 为 -8。
        本方法只使用原始计数方向；软件方向 self.direction 在换算时乘一次。
        """
        if not 0 <= position < self.encoder_resolution:
            raise ValueError(
                f"单圈位置超出范围: position={position}, "
                f"resolution={self.encoder_resolution}，请核对分辨率和寄存器"
            )

        if self.last_position is None:
            self.last_position = position
            self.last_direction = 0
            return None

        diff = position - self.last_position
        self.last_position = position
        half_resolution = self.encoder_resolution / 2.0

        if abs(diff) == half_resolution:
            # 正反转半圈会得到相同的单圈位置，不能据此选择方向。
            self.last_direction = 0
            self.get_logger().warning(
                "相邻位置相差半圈，方向不确定；本段位移未累计，已重建位置基准。"
                "请提高有效读取频率或降低转速。",
                throttle_duration_sec=self.info_time_s,
            )
            return None

        if diff > half_resolution:
            diff -= self.encoder_resolution
        elif diff < -half_resolution:
            diff += self.encoder_resolution

        if diff > 0:
            self.last_direction = 1
        elif diff < 0:
            self.last_direction = -1
        # diff == 0 时，和 encoder_1 一样保持上一次有效方向。
        return diff

    # 原始计数 -> SI 单位
    def convert_to_si(self, speed_raw, position, motion_direction):
        encoder_sample_s = self.encoder_speed_sample_ms / 1000.0

        # 速度寄存器只提供大小，位置差提供方向。
        # 必须先 abs()，避免速度值本身为负时再乘 -1，变成正数。
        signed_speed_raw = abs(speed_raw) * motion_direction * self.direction
        angular_velocity = (
            signed_speed_raw
            / self.encoder_resolution
            / encoder_sample_s
            * 2.0
            * math.pi
        )

        # 线速度：rev/s * wheel_circumference -> m/s。
        speed = (
            signed_speed_raw
            / self.encoder_resolution
            / encoder_sample_s
            * self.wheel_circumference
        )

        # 低速死区：±1 个编码器计数认为是静止抖动。
        if abs(speed_raw) <= 1:
            angular_velocity = 0.0
            speed = 0.0

        # 角度：count -> rev -> rad。
        # self.direction=-1 时角度符号也随软件方向修正。
        angle = (
            float(position)
            / self.encoder_resolution
            * 2.0
            * math.pi
            * self.direction
        )

        return angular_velocity, angle, speed

    # 定时读取并发布
    def timer_callback(self):
        try:
            # 速度寄存器是送入 EKF 的实际测量量。读取完成后立即记录主机
            # SYSTEM_TIME，避免后续位置寄存器查询的 Modbus 延迟污染速度时间戳。
            speed_raw = self.read_speed_value()
            speed_response_ns = self.system_clock.now().nanoseconds
            position = self.read_position()

            if speed_raw is None or position is None:
                self.get_logger().warning(
                    "编码器数据读取失败",
                    throttle_duration_sec=self.info_time_s,
                )
                return

            now = time.monotonic()
            delta_count = self.get_position_delta(position)

            (
                self.angular_velocity,
                self.angle,
                self.speed,
            ) = self.convert_to_si(speed_raw, position, self.last_direction)

            # 用带符号的位置增量累计位移，避免速度采样窗口与 ROS 查询
            # 不同步造成积分误差。速度死区不丢弃有效的位置计数。
            if delta_count is not None:
                self.distance += (
                    delta_count
                    / self.encoder_resolution
                    * self.wheel_circumference
                    * self.direction
                )

            # 加速度仍使用两次有效速度样本之间的真实时间间隔。
            if (
                delta_count is None
                or self.last_time is None
                or self.last_speed is None
            ):
                self.acceleration = 0.0
            else:
                dt = now - self.last_time
                if dt > 0.0:
                    self.acceleration = (self.speed - self.last_speed) / dt
                else:
                    self.acceleration = 0.0

            # 首帧/方向不确定帧的速度为 0，但不是有效的测速基准。
            self.last_time = now if delta_count is not None else None
            self.last_speed = self.speed if delta_count is not None else None

            # 编码器速度是 encoder_speed_sample_ms 时间窗上的平均值，因此
            # header.stamp 取该时间窗的中心，而不是第二次 Modbus 查询结束后的
            # “发布时刻”。这样 /odom 与 IMU 的 stamp 都表示各自的测量时刻。
            sample_half_window_ns = int(self.encoder_speed_sample_ms * 0.5 * 1_000_000)
            measurement_stamp_ns = speed_response_ns - sample_half_window_ns

            # 正常情况下 SYSTEM_TIME 单调前进；这里仍做保护，避免系统校时或
            # 异常调度让同一传感器产生非递增 stamp，robot_localization 对这种
            # 时间倒退非常敏感。
            if self.last_stamp_ns is not None and measurement_stamp_ns <= self.last_stamp_ns:
                measurement_stamp_ns = self.last_stamp_ns + 1
            self.last_stamp_ns = measurement_stamp_ns

            msg = Vector3Stamped()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
                measurement_stamp_ns, 1_000_000_000
            )
            msg.vector.x = float(self.speed)  # Vx，m/s
            msg.vector.y = 0.0                # Vy，m/s
            msg.vector.z = 0.0                # Vz，m/s

            self.speed_pub.publish(msg)

            self.get_logger().info(
                f"Vx:{msg.vector.x:.3f} m/s "
                f"Vy:{msg.vector.y:.3f} m/s "
                f"Vz:{msg.vector.z:.3f} m/s "
                f"角速度:{self.angular_velocity:.3f} rad/s "
                f"加速度:{self.acceleration:.3f} m/s^2 "
                f"角度:{self.angle:.3f} rad "
                f"距离:{self.distance:.3f} m "
                f"速度原值:{speed_raw} "
                f"位置:{position} "
                f"位置增量:{delta_count} "
                f"计数方向:{self.last_direction:+d} "
                f"时间戳:{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
                f"(速度采样窗中心)",
                throttle_duration_sec=self.info_time_s,
            )

        except Exception as e:
            self.get_logger().error(
                f"encoder_raw 运行异常: {e}",
                throttle_duration_sec=self.info_time_s,
            )

    def destroy_node(self):
        if hasattr(self, "ser") and self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = EncoderRawNode()

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
