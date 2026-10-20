#!/usr/bin/env python3
"""ROS 2 小车底盘驱动。

代码按功能分为 5 个区域：
1. STM32 串口通信：打开串口、封装并发送 PWM、关闭串口。
2. ROS 2 驱动节点初始化：参数、状态和 ROS 接口。
3. ROS 2 输入回调：接收电机 PWM 和舵机 PWM，并直接发送。
4. 安全停车与资源释放。
5. 程序入口与 SIGINT / SIGTERM 退出处理。

"""

import errno
import math
import os
import signal
import struct
import termios
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


# ============================================================================
# 1. STM32 串口通信
#    与 STM32 串口有关的打开、数据帧封装、发送、关闭都集中在这个类中。
# ============================================================================
class RacecarSerial:
    """负责与 STM32 进行串口通信。"""

    def __init__(self, baud_rate: int = 38400, serial_port: str = "/dev/car") -> None:
        self.baud_rate = int(baud_rate)
        self.serial_port = str(serial_port)

        # Linux 串口文件描述符；None 表示串口尚未打开。
        self.fd: Optional[int] = None

        # 定时器和 ROS 回调都可能访问串口，用锁避免并发写入。
        self._lock = threading.RLock()

        # 创建对象时直接打开串口。
        self.open()

    def open(self) -> None:
        """打开串口，并配置为 8N1、无硬件流控。"""
        if self.fd is not None:
            return

        # termios 用 B38400、B115200 等常量表示波特率。
        speed_name = f"B{self.baud_rate}"
        if not hasattr(termios, speed_name):
            raise ValueError(f"Unsupported baud rate: {self.baud_rate}")
        speed = getattr(termios, speed_name)

        # O_RDWR  ：允许读写。
        # O_NOCTTY：不让该串口成为当前进程的控制终端。
        fd = os.open(self.serial_port, os.O_RDWR | os.O_NOCTTY)

        try:
            attrs = termios.tcgetattr(fd)

            # 使用接近原始字节流的方式通信，不进行字符转换。
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            attrs[3] = 0

            # 设置输入、输出波特率。
            attrs[4] = speed
            attrs[5] = speed

            # 本节点主要负责向 STM32 发送数据，因此读取设置为非阻塞。
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0

            # 8N1：8 位数据位、无校验、1 位停止位。
            attrs[2] &= ~termios.PARENB
            attrs[2] &= ~termios.CSTOPB

            # 关闭硬件流控。
            if hasattr(termios, "CRTSCTS"):
                attrs[2] &= ~termios.CRTSCTS

            termios.tcflush(fd, termios.TCIOFLUSH)
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except Exception:
            os.close(fd)
            raise

        self.fd = fd

    def send_cmd(self, motor_pwm: int, servo_pwm: int) -> None:
        """封装并发送一帧电机 PWM 和舵机 PWM 给 STM32。"""
        # 电机和舵机 PWM 均按 uint16 发送。
        motor = int(motor_pwm) & 0xFFFF
        servo = int(servo_pwm) & 0xFFFF

        # 校验和：motor 和 servo 共 4 个数据字节相加，保留低 8 位。
        checksum = (
            (motor & 0xFF)
            + ((motor >> 8) & 0xFF)
            + (servo & 0xFF)
            + ((servo >> 8) & 0xFF)
        ) & 0xFF

        # STM32 控制帧共 8 字节，小端格式：
        # Byte0   : 0x00
        # Byte1   : 0xAA
        # Byte2-3 : motor_pwm
        # Byte4-5 : servo_pwm
        # Byte6   : checksum
        # Byte7   : 0x55
        frame = struct.pack(
            "<BBHHBB",
            0x00,
            0xAA,
            motor,
            servo,
            checksum,
            0x55,
        )

        with self._lock:
            if self.fd is None:
                raise OSError(errno.ENODEV, "Serial port is not open")

            # os.write() 不保证一次写完全部数据，所以循环发送直到整帧写完。
            total = 0
            while total < len(frame):
                try:
                    written = os.write(self.fd, frame[total:])
                except InterruptedError:
                    continue

                if written <= 0:
                    raise OSError(errno.EIO, "Serial write returned 0 bytes")
                total += written

    def close(self) -> None:
        """关闭串口。"""
        with self._lock:
            if self.fd is None:
                return

            try:
                os.close(self.fd)
            finally:
                self.fd = None


# ============================================================================
# 2. ROS 2 驱动节点：串口、输出状态与 ROS 接口初始化
#    数据流：/cmd_vel.linear.x -> 电机 PWM；
#           /cmd_vel.angular.z -> 舵机 PWM。
# ============================================================================
class RacecarDriver(Node):
    """接收电机 PWM 和舵机 PWM，并直接发送给 STM32。"""

    def __init__(self) -> None:
        super().__init__("racecar_driver")

        # --------------------------------------------------------------------
        # 串口配置
        # --------------------------------------------------------------------
        # 串口配置直接在代码中指定。
        serial_port = "/dev/car"
        baud_rate = 38400

        # --------------------------------------------------------------------
        # STM32 串口
        # --------------------------------------------------------------------
        self.serial = RacecarSerial(baud_rate, serial_port)

        # --------------------------------------------------------------------
        # 当前输出状态
        # --------------------------------------------------------------------
        self._motor_pwm = 1500
        self._servo_pwm = 1500

        # --------------------------------------------------------------------
        # ROS 2 接口
        # --------------------------------------------------------------------
        # 控制只关心最新值，因此队列深度设为 1。
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # /cmd_vel：linear.x=电机 PWM，angular.z=舵机 PWM。
        self.create_subscription(Twist, "/cmd_vel", self.twist_callback, qos)

        self.get_logger().info(
            f"racecar_driver 已启动：串口={serial_port}, "
            f"波特率={baud_rate}, 控制方式=直接 PWM"
        )

    # ========================================================================
    # 3. ROS 2 输入回调
    # ========================================================================
    def twist_callback(self, msg: Twist) -> None:
        """接收电机 PWM 和舵机 PWM，并立即发送。"""
        motor = float(msg.linear.x)
        servo = float(msg.angular.z)

        if not (math.isfinite(motor) and math.isfinite(servo)):
            self.get_logger().error(
                "无效控制命令：linear.x 和 angular.z 必须是有限数值",
                throttle_duration_sec=1.0,
            )
            return

        # 输入值直接作为 PWM 使用，只做输出范围限幅。
        self._motor_pwm = int(round(min(max(motor, 1320.0), 1680.0)))
        self._servo_pwm = int(round(min(max(servo, 1100.0), 1900.0)))

        try:
            self.serial.send_cmd(self._motor_pwm, self._servo_pwm)
        except Exception as exc:
            self.get_logger().error(
                f"串口发送失败: {type(exc).__name__}: {exc}",
                throttle_duration_sec=1.0,
            )

    # ========================================================================
    # 4. 安全停车与资源释放
    # ========================================================================
    def stop_vehicle(self, count: int = 5, interval: float = 0.05) -> None:
        """退出前连续发送停车和舵机回中命令。"""
        # 多发几帧，降低某一帧未被 STM32 正确接收造成的风险。
        for _ in range(count):
            try:
                # motor=1500：电机停止；servo=1500：舵机回中。
                self.serial.send_cmd(1500, 1500)
            except Exception:
                break
            time.sleep(interval)

    def destroy_node(self) -> bool:
        """关闭底层串口，再销毁 ROS 节点。"""
        try:
            self.serial.close()
        finally:
            return super().destroy_node()


# ============================================================================
# 5. 程序入口与退出信号处理
# ============================================================================
def main(args=None) -> None:
    """启动 ROS 2 节点，并处理 Ctrl+C / SIGTERM 安全停车。"""
    rclpy.init(args=args)
    node = RacecarDriver()

    def emergency_stop(sig, _frame) -> None:
        """收到退出信号时先停车并让舵机回中。"""
        signal_name = signal.Signals(sig).name
        node.get_logger().warning(
            f">>> 收到 {signal_name}，正在强制停车并让舵机回中... <<<"
        )

        node.stop_vehicle(count=5, interval=0.05)
        raise KeyboardInterrupt

    # Ctrl+C 通常产生 SIGINT；
    # ros2 launch、systemd 或 kill 通常会使用 SIGTERM。
    signal.signal(signal.SIGINT, emergency_stop)
    signal.signal(signal.SIGTERM, emergency_stop)

    try:
        # 持续处理 /cmd_vel；每条新消息直接发送电机和舵机 PWM。
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
