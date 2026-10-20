#!/usr/bin/env python3
"""ROS 2 小车串口驱动，包含多层失效保护（failsafe）。

安全链路说明：
1. ROS 话题 /cmd_vel 只负责“更新目标 PWM”，本节点并不会按 /cmd_vel 的接收频率直接发串口。
2. 如果超过 command_timeout_sec（默认 0.5 s）没有收到新的 ROS 控制命令，
   就把目标值强制改成停车/回中：motor=1500, servo=1500。
3. 无论 /cmd_vel 来得快还是慢，都会由 serial_heartbeat 定时器每 0.02 s
   将“当前目标值”发送给 STM32，即 50 Hz 串口控制/心跳频率。
4. 收到 SIGINT / SIGTERM / SIGHUP 时，会尽量连续发送若干停车帧后再退出。
5. 如果进程被 SIGKILL 强杀，Python 无法执行退出逻辑，此时串口心跳会消失，
   需要 STM32 端依靠自己的通信超时（示例固件中为 300 ms）执行停车。
"""

import errno
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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class RacecarSerial:
    """负责底层串口打开、配置、组帧、发送和自动断线标记。"""

    # 这些错误通常说明串口设备暂时不可用/已经断开，发生后把 fd 关闭，
    # 交给上层的 reconnect_timer 周期性尝试重连。
    RECOVERABLE_ERRORS = {
        errno.EIO,
        errno.ENODEV,
        errno.ENXIO,
        errno.EBADF,
        errno.EPIPE,
    }

    def __init__(self, baud_rate: int = 38400, serial_port: str = "/dev/car") -> None:
        # 串口波特率，默认 38400 bit/s。
        self.baud_rate = int(baud_rate)
        # Linux 下的串口设备路径。
        self.serial_port = str(serial_port)
        # 文件描述符；None 表示当前未打开。
        self.fd: Optional[int] = None
        # 串口发送/开关操作可能来自多个回调，使用可重入锁保护。
        self._lock = threading.RLock()

        # 启动时先尝试打开串口；失败也不让节点直接退出，后面会自动重连。
        try:
            self.open()
        except Exception:
            self._close_unlocked()

    def open(self) -> None:
        """打开并配置串口。若已经打开则直接返回。"""
        if self.fd is not None:
            return

        # O_NOCTTY：不要让该串口成为进程的控制终端。
        fd = os.open(self.serial_port, os.O_RDWR | os.O_NOCTTY)
        try:
            self._configure(fd, self.baud_rate)
        except Exception:
            # 配置失败时必须关闭刚刚打开的 fd，避免资源泄漏。
            try:
                os.close(fd)
            except OSError:
                pass
            raise
        self.fd = fd

    @staticmethod
    def _configure(fd: int, baud_rate: int) -> None:
        """将串口配置为 8N1、无流控、指定波特率。"""
        speed_name = f"B{baud_rate}"
        if not hasattr(termios, speed_name):
            raise ValueError(f"Unsupported baud rate: {baud_rate}")
        speed = getattr(termios, speed_name)

        attrs = termios.tcgetattr(fd)

        # 输入模式/输出模式/本地模式都尽量关闭额外处理，按原始字节通信。
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0

        # 设置输入/输出波特率。
        attrs[4] = speed
        attrs[5] = speed

        # 非阻塞式读取参数；本程序主要向 STM32 写数据。
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0

        # 8N1：无校验位、1 个停止位。
        attrs[2] &= ~termios.PARENB
        attrs[2] &= ~termios.CSTOPB

        # 关闭 RTS/CTS 硬件流控（如果系统支持该标志）。
        if hasattr(termios, "CRTSCTS"):
            attrs[2] &= ~termios.CRTSCTS

        # 清空串口缓存，并立即应用配置。
        termios.tcflush(fd, termios.TCIOFLUSH)
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    @staticmethod
    def build_frame(motor_pwm: int, servo_pwm: int) -> bytes:
        """构造发送给 STM32 的 8 字节控制帧。

        帧格式（小端）：
            Byte 0 : 0x00
            Byte 1 : 0xAA              帧头
            Byte 2-3 : motor_pwm       uint16 little-endian
            Byte 4-5 : servo_pwm       uint16 little-endian
            Byte 6 : checksum          motor/servo 四个数据字节求和后取低 8 位
            Byte 7 : 0x55              帧尾
        """
        # 限定为 16 位无符号数的字节表示。
        motor = int(motor_pwm) & 0xFFFF
        servo = int(servo_pwm) & 0xFFFF

        # 对 motor 和 servo 的 4 个数据字节求和，保留最低 8 位作为校验和。
        checksum = (
            (motor & 0xFF)
            + ((motor >> 8) & 0xFF)
            + (servo & 0xFF)
            + ((servo >> 8) & 0xFF)
        ) & 0xFF

        # < 表示小端；BBHHBB 共 8 字节。
        return struct.pack("<BBHHBB", 0x00, 0xAA, motor, servo, checksum, 0x55)

    def _close_unlocked(self) -> None:
        """内部关闭串口。调用者负责外部加锁。"""
        if self.fd is None:
            return
        try:
            os.close(self.fd)
        except OSError:
            pass
        self.fd = None

    def _write_frame(self, frame: bytes) -> None:
        """确保一整帧数据全部写入串口。"""
        if self.fd is None:
            raise OSError(errno.ENODEV, "Serial port is not open")

        total = 0
        while total < len(frame):
            try:
                # os.write 可能只写入部分字节，因此循环直到整帧发完。
                written = os.write(self.fd, frame[total:])
            except InterruptedError:
                # 被信号暂时打断时重试，不认为是串口故障。
                continue

            if written <= 0:
                raise OSError(errno.EIO, "Serial write returned 0 bytes")
            total += written

    def is_open(self) -> bool:
        """查询串口当前是否处于打开状态。"""
        with self._lock:
            return self.fd is not None

    def reconnect_once(self) -> bool:
        """执行一次重连尝试；成功返回 True，失败返回 False。"""
        with self._lock:
            if self.fd is not None:
                return True
            try:
                self.open()
                return True
            except Exception:
                self._close_unlocked()
                return False

    def send_cmd(self, motor_pwm: int, servo_pwm: int) -> int:
        """构造并发送一帧电机/舵机 PWM 命令。"""
        frame = self.build_frame(motor_pwm, servo_pwm)

        with self._lock:
            if self.fd is None:
                raise OSError(errno.ENODEV, "Serial port is disconnected")

            try:
                self._write_frame(frame)
            except OSError as exc:
                # 如果是典型断线错误，立即关闭 fd。
                # 上层 reconnect_check() 会以 5 Hz 频率尝试重新打开。
                if exc.errno in self.RECOVERABLE_ERRORS:
                    self._close_unlocked()
                raise

        # 返回校验和字节；当前上层没有使用这个返回值。
        return frame[-2]

    def close(self) -> None:
        """线程安全地关闭串口。"""
        with self._lock:
            self._close_unlocked()


class RacecarDriver(Node):
    """ROS 2 节点：接收 /cmd_vel，并以固定 50 Hz 向 STM32 发当前安全控制值。"""

    def __init__(self) -> None:
        super().__init__("racecar_driver")

        # ------------------------- ROS 参数 -------------------------
        # 串口设备，默认通过 udev 映射为 /dev/car。
        self.declare_parameter("serial_port", "/dev/car")
        # 串口波特率。
        self.declare_parameter("baud_rate", 38400)
        # ROS 控制命令超时时间：0.5 s 内收不到新的 /cmd_vel 就停车。
        self.declare_parameter("command_timeout_sec", 0.5)

        self.serial_port = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )

        # 创建底层串口对象。即使启动时串口不存在，也允许节点继续运行等待重连。
        self.serial = RacecarSerial(self.baud_rate, self.serial_port)
        # 防止串口持续故障时每 20 ms 刷一条错误日志。
        self.serial_error_reported = False

        # ------------------------- 当前控制状态 -------------------------
        # 多个 ROS 定时器/回调都会读写这些变量，因此加锁保护。
        self._state_lock = threading.Lock()

        # 默认值 1500/1500：通常表示电机停车、舵机回中。
        self._desired_motor = 1500
        self._desired_servo = 1500

        # 使用 monotonic() 计时，不受系统时间校准/跳变影响。
        self._last_cmd_monotonic = time.monotonic()

        # 是否至少收到过一条有效 /cmd_vel。
        self._have_control_cmd = False
        # 当前是否已经进入 ROS 指令超时停车状态。
        self._timed_out = True

        # ------------------------- ROS 订阅 -------------------------
        # 接收上游控制命令。
        # 注意：这里没有“固定接收频率”，回调频率取决于 /cmd_vel 发布者。
        # QoS depth=1：只保留最新一条命令，适合实时控制，避免积压旧指令。
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.twist_callback, qos_profile
        )

        # ------------------------- 三个定时器 -------------------------
        # 1) ROS 命令超时检查：0.05 s 一次 = 20 Hz。
        #    它只负责判断 /cmd_vel 是否超时，不直接形成正常控制发送频率。
        self.watchdog_timer = self.create_timer(
            0.05, self.timeout_check
        )

        # 2) 底盘串口心跳/控制发送：0.02 s 一次 = 50 Hz。
        #    这就是“真正发布到底盘 STM32”的固定频率。
        #    即使 /cmd_vel 没有更新，也会重复发送当前安全值。
        self.heartbeat_timer = self.create_timer(
            0.02, self.serial_heartbeat
        )

        # 3) 串口重连检查：0.2 s 一次 = 5 Hz。
        self.reconnect_timer = self.create_timer(
            0.2, self.reconnect_check
        )

        # 如果启动时串口已经连通，先发 5 帧停车命令。
        # 每帧间隔默认 0.02 s，因此这段停车 burst 也是按 50 Hz 发。
        if self.serial.is_open():
            self.send_stop_frames(count=5)
            self.get_logger().info("初始化完成：电机停车，舵机回中")
        else:
            self.get_logger().warn(
                f"启动时串口 {self.serial_port} 未连接，等待自动重连"
            )

        self.get_logger().info(
            f"racecar_driver 已启动：串口={self.serial_port}, 波特率={self.baud_rate}, "
            f"ROS控制超时={self.command_timeout_sec:.3f}s, 串口心跳=50Hz"
        )

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        """把输入限制在 [minimum, maximum] 范围内。"""
        return min(max(value, minimum), maximum)

    def _set_desired(self, motor_pwm: int, servo_pwm: int) -> None:
        """保存最新 ROS 控制目标，并刷新“最后收到控制命令”的时间。"""
        with self._state_lock:
            self._desired_motor = int(motor_pwm)
            self._desired_servo = int(servo_pwm)
            self._last_cmd_monotonic = time.monotonic()
            self._have_control_cmd = True
            self._timed_out = False

    def _set_stop_due_timeout(self) -> None:
        """ROS 控制超时时，把当前目标锁定为停车/回中。"""
        with self._state_lock:
            self._desired_motor = 1500
            self._desired_servo = 1500
            self._timed_out = True

    def safe_send_cmd(self, motor_pwm: int, servo_pwm: int) -> bool:
        """安全发送一帧；串口异常时只在首次失败时打印日志，避免刷屏。"""
        try:
            self.serial.send_cmd(int(motor_pwm), int(servo_pwm))
            self.serial_error_reported = False
            return True
        except Exception as exc:
            if not self.serial_error_reported:
                self.get_logger().error(
                    f"串口发送失败: {type(exc).__name__}: {exc}"
                )
                self.serial_error_reported = True
            return False

    def reconnect_check(self) -> None:
        """每 0.2 s（5 Hz）检查一次断开的串口，并尝试重连。"""
        # 串口正常时无需任何操作。
        if self.serial.is_open():
            return

        # 本次重连失败，等待下一个 0.2 s 周期继续尝试。
        if not self.serial.reconnect_once():
            return

        self.get_logger().warn(f"串口 {self.serial_port} 已重新连接")

        # 重要安全策略：
        # 重连成功后绝不恢复断线前的旧油门，必须等新的 ROS /cmd_vel。
        with self._state_lock:
            self._desired_motor = 1500
            self._desired_servo = 1500
            self._have_control_cmd = False
            self._timed_out = True
            self._last_cmd_monotonic = time.monotonic()

        # 重连后先连续发 5 帧停车/回中命令。
        self.send_stop_frames(count=5)

    def twist_callback(self, twist: Twist) -> None:
        """收到 /cmd_vel 后更新目标 PWM，但这里不直接写串口。"""
        # 这里把 Twist.linear.x 当作电机 PWM 使用，而不是常规意义上的 m/s。
        # 允许范围：500 ~ 2500。
        motor_pwm = self._clamp(
            float(twist.linear.x), 500, 2500
        )

        # 这里把 Twist.angular.z 当作舵机 PWM 使用，而不是常规意义上的 rad/s。
        # 允许范围：1100 ~ 1900。
        servo_pwm = self._clamp(
            float(twist.angular.z), 1100, 1900
        )

        # 只更新“当前目标值”。真正发给 STM32 的动作由 50 Hz heartbeat_timer 完成。
        self._set_desired(int(motor_pwm), int(servo_pwm))

    def timeout_check(self) -> None:
        """20 Hz 检查 ROS /cmd_vel 是否超过 command_timeout_sec 未更新。"""
        with self._state_lock:
            have_cmd = self._have_control_cmd
            timed_out = self._timed_out
            elapsed = time.monotonic() - self._last_cmd_monotonic

        # 从未收到控制命令，或者已经处理过超时，就不用重复处理。
        if not have_cmd or timed_out:
            return

        # 默认 command_timeout_sec=0.5 s。
        # 一旦超时，把“目标输出”改成停车/回中；
        # 后续 50 Hz 串口心跳会持续把 1500/1500 发给 STM32。
        if elapsed > self.command_timeout_sec:
            self.get_logger().error(
                f">>> ROS 控制通信超时 {elapsed:.3f}s：锁定当前输出为 1500/1500 <<<"
            )
            self._set_stop_due_timeout()

    def serial_heartbeat(self) -> None:
        """50 Hz（每 20 ms）将当前安全输出发送给 STM32。"""
        # 只在锁内快速复制状态，串口 I/O 放到锁外，减少对控制状态更新的阻塞。
        with self._state_lock:
            motor_pwm = self._desired_motor
            servo_pwm = self._desired_servo

        # 无论目标值有没有变化，每 20 ms 都会发一帧。
        # 因此 STM32 可以把“持续收到控制帧”同时作为上位机存活心跳。
        self.safe_send_cmd(motor_pwm, servo_pwm)

    def send_stop_frames(self, count: int = 8, interval: float = 0.02) -> None:
        """连续发送若干停车帧；默认 8 帧、间隔 20 ms（50 Hz）。"""
        if not self.serial.is_open():
            return

        for _ in range(count):
            try:
                # 1500/1500 = 电机停车 + 舵机回中。
                self.serial.send_cmd(1500, 1500)
            except Exception:
                # 串口已经不可用时不继续发送。
                break

            # 默认 0.02 s，对应 50 Hz。
            time.sleep(interval)

    def destroy_node(self) -> bool:
        """节点销毁前尽量先发送停车帧，然后关闭串口。"""
        try:
            # 默认发送 8 帧停车命令，大约持续 8*20 ms = 160 ms
            # （不考虑函数执行本身的少量开销）。
            self.send_stop_frames()
        finally:
            try:
                self.serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    """ROS 2 节点主入口。"""
    rclpy.init(args=args)
    node: Optional[RacecarDriver] = None

    try:
        node = RacecarDriver()

        def emergency_stop(sig, _frame) -> None:
            """收到退出相关信号时，先尽量发送停车帧，再退出 spin。"""
            name = signal.Signals(sig).name

            if node is not None:
                try:
                    node.get_logger().error(
                        f">>> 收到 {name}，立即发送停车帧 <<<"
                    )
                    node.send_stop_frames()
                except Exception:
                    pass

            # 统一走 KeyboardInterrupt -> finally 的正常清理路径。
            raise KeyboardInterrupt

        # Ctrl+C 通常产生 SIGINT。
        signal.signal(signal.SIGINT, emergency_stop)
        # systemd / kill 默认通常产生 SIGTERM。
        signal.signal(signal.SIGTERM, emergency_stop)
        # 终端挂断时可能产生 SIGHUP。
        if hasattr(signal, "SIGHUP"):
            signal.signal(signal.SIGHUP, emergency_stop)

        # 进入 ROS 事件循环，处理 /cmd_vel 回调和各个定时器。
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                # destroy_node() 内还会再次尝试发送停车帧并关闭串口。
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
