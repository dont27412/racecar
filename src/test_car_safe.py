#!/usr/bin/env python3
"""ROS 2 键盘遥控程序，带终端/SSH断连安全停车功能。

底盘通信：
- 发布到 /cmd_vel，与底盘驱动的话题保持一致。
- Twist.linear.x 直接传电机 PWM，Twist.angular.z 直接传舵机 PWM。
- QoS 使用 BEST_EFFORT、KEEP_LAST、depth=1。

安全逻辑：
- MobaXterm/SSH 终端正常连接时，以 20 Hz（每 0.05 秒）持续发布当前 PWM。
- 如果 stdin 检测到 EOF/HUP/ERR（常见于 SSH/MobaXterm 断开），立即锁定停车。
- 安全停车时电机 PWM=1500、舵机 PWM=1500。
- 安全锁定后持续发布 1500/1500，并忽略后续按键，必须重启节点才能恢复控制。
- 收到 SIGINT/SIGTERM/SIGHUP 时，也会在退出前快速发送多次停车命令。
- 舵机安全范围建议：最小 1100、中位 1500、最大 1900。
"""

import os
import select
import signal
import sys
import termios
import threading
import time
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class TestCar(Node):
    # 停车/回中值，同时用于启动初值和安全停车命令。
    STOP_PWM = 1500.0
    # ROS 命令发布周期：0.05 秒，即 20 Hz；不是底盘串口发送周期。
    PUBLISH_PERIOD_SEC = 0.05

    def __init__(self) -> None:
        super().__init__("test_car")

        # 与底盘订阅端保持相同的 QoS 配置。
        qos_profile = QoSProfile(
            # 尽力而为：允许丢包，不要求通过重传保证每条命令送达。
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            # 仅保留最近的消息，保留数量由 depth 指定。
            history=QoSHistoryPolicy.KEEP_LAST,
            # 历史队列深度为 1；此处不设置发布频率。
            depth=1,
        )
        # 普通控制命令和停车命令都通过此发布器发送到 /cmd_vel。
        self.publisher = self.create_publisher(Twist, "/cmd_vel", qos_profile)

        # 启动时先停车、回中，等待键盘操作。
        self.motor_pwm = self.STOP_PWM
        self.servo_pwm = self.STOP_PWM
        self.motor_min = 500.0
        self.motor_max = 1700.0  # 键盘控制的电机上限保持为 1700。
        # 舵机安全限幅：1100（左/一端）～1900（右/另一端），1500 为中位
        # 如实际机械结构方向相反，只影响转向方向，不影响限幅保护
        self.servo_min = 1100.0
        self.servo_max = 1900.0
        self.step = 100.0  # 每次转向按键调整的舵机 PWM。
        self.motor_step = 50.0  # 每次电机按键调整的电机 PWM。

        # 键盘线程、SSH 监控线程和 ROS 定时器共享控制状态，用锁保护。
        self._state_lock = threading.Lock()
        self._key: str = ""
        self._safety_latched = False
        self._safety_reason = ""
        self._stop_event = threading.Event()

        self.stdin_fd = sys.stdin.fileno()
        if not os.isatty(self.stdin_fd):
            raise RuntimeError(
                "test_car 必须从交互终端启动；当前 stdin 不是 TTY，拒绝进入可运动状态"
            )

        # 保存原终端配置；cbreak 模式下按键无需回车即可读取。
        self.settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)

        # SSH_CONNECTION 格式："客户端IP 客户端端口 服务器IP 服务器端口"。
        # 除了监测TTY HUP/EOF，再直接监测这条SSH TCP连接是否还存在。
        self._ssh_client_port = None
        self._ssh_server_port = None
        ssh_connection = os.environ.get("SSH_CONNECTION", "").split()
        if len(ssh_connection) == 4:
            try:
                self._ssh_client_port = int(ssh_connection[1])
                self._ssh_server_port = int(ssh_connection[3])
            except ValueError:
                self._ssh_client_port = None
                self._ssh_server_port = None

        self.key_thread = threading.Thread(
            target=self.keyboard_thread,
            name="teleop-keyboard",
            daemon=True,
        )
        self.key_thread.start()

        self.ssh_thread = None
        if self._ssh_client_port is not None and self._ssh_server_port is not None:
            self.ssh_thread = threading.Thread(
                target=self.ssh_monitor_thread,
                name="teleop-ssh-monitor",
                daemon=True,
            )
            self.ssh_thread.start()

        # 即使没有新按键，也持续发布当前 PWM，刷新底盘的控制超时计时。
        self.timer = self.create_timer(self.PUBLISH_PERIOD_SEC, self.publish_cmd)

        self.get_logger().info(
            "控制说明：w 电机+50 | s 电机-50 | j 舵机-100 | "
            "k 舵机+100 | i 舵机回中 | q 停车回中。电机上限 1700；舵机限幅 1100~1900，中位 1500。终端/SSH断开会安全锁定停车。"
        )

    def latch_stop(self, reason: str) -> None:
        """锁定停车。锁定后必须重启本节点才能恢复运动。"""
        first = False
        with self._state_lock:
            if not self._safety_latched:
                self._safety_latched = True
                self._safety_reason = reason
                first = True
            self.motor_pwm = self.STOP_PWM
            self.servo_pwm = self.STOP_PWM
            self._key = ""

        if first:
            self.get_logger().error(
                f">>> 安全停车已锁定：{reason}；必须重启 test_car 才能恢复 <<<"
            )

    def keyboard_thread(self) -> None:
        """监听按键，同时直接监测终端 HUP/EOF。"""
        poller = select.poll()
        poller.register(
            self.stdin_fd,
            select.POLLIN | select.POLLHUP | select.POLLERR | select.POLLNVAL,
        )

        while not self._stop_event.is_set():
            try:
                events = poller.poll(200)
            except Exception as exc:
                self.latch_stop(f"终端监听异常: {type(exc).__name__}: {exc}")
                return

            for _fd, event in events:
                if event & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                    self.latch_stop("MobaXterm/SSH 终端已断开(HUP/ERR)")
                    return

                if event & select.POLLIN:
                    try:
                        data = os.read(self.stdin_fd, 1)
                    except OSError as exc:
                        self.latch_stop(f"终端读取失败: {exc}")
                        return

                    if not data:
                        self.latch_stop("MobaXterm/SSH 终端 EOF")
                        return

                    try:
                        key = data.decode(errors="ignore")
                    except Exception:
                        key = ""

                    if key:
                        with self._state_lock:
                            if not self._safety_latched:
                                self._key = key

    def _ssh_connection_alive(self) -> bool:
        """从 /proc/net/tcp{,6} 检查启动本节点的 SSH TCP 会话。"""
        if self._ssh_client_port is None or self._ssh_server_port is None:
            return True

        for path in ("/proc/net/tcp", "/proc/net/tcp6"):
            try:
                with open(path, "r", encoding="ascii", errors="ignore") as f:
                    next(f, None)
                    for line in f:
                        fields = line.split()
                        if len(fields) < 4 or fields[3] != "01":  # "01" 表示 TCP 已建立连接状态
                            continue
                        try:
                            local_port = int(fields[1].split(":")[1], 16)
                            remote_port = int(fields[2].split(":")[1], 16)
                        except (IndexError, ValueError):
                            continue
                        if (
                            local_port == self._ssh_server_port
                            and remote_port == self._ssh_client_port
                        ):
                            return True
            except OSError:
                continue
        return False

    def ssh_monitor_thread(self) -> None:
        """直接监视最初启动本节点的 SSH 连接。"""
        # 给启动阶段一点时间，避免刚创建线程时 /proc 状态瞬变。
        time.sleep(0.5)
        missing_count = 0
        while not self._stop_event.wait(0.25):
            if self._ssh_connection_alive():
                missing_count = 0
                continue

            # 连续两次不存在再锁定，过滤瞬时读取异常。
            missing_count += 1
            if missing_count >= 2:
                self.latch_stop("启动本节点的 MobaXterm/SSH TCP 会话已断开")
                return

    def _make_msg(self, motor_pwm: float, servo_pwm: float) -> Twist:
        """构造底盘命令：两个字段直接携带 PWM，不进行速度换算。"""
        msg = Twist()
        # 此驱动约定 linear.x 为电机 PWM，不是常规 Twist 的线速度 m/s。
        msg.linear.x = float(motor_pwm)
        # 此驱动约定 angular.z 为舵机 PWM，不是常规 Twist 的角速度 rad/s。
        msg.angular.z = float(servo_pwm)
        return msg

    def publish_stop_once(self) -> None:
        """尝试发布一次 1500/1500 停车、回中命令。"""
        try:
            self.publisher.publish(self._make_msg(self.STOP_PWM, self.STOP_PWM))
        except Exception:
            pass

    def publish_stop_burst(self, count: int = 8, interval: float = 0.03) -> None:
        """退出前重复发送停车命令；尽力而为模式不保证每条都送达。"""
        for _ in range(count):
            self.publish_stop_once()
            time.sleep(interval)

    def publish_cmd(self) -> None:
        """每 0.05 秒处理一次记录的按键，并发布当前 PWM。"""
        with self._state_lock:
            if self._safety_latched:
                # 关键：断连后不再持续发布旧 PWM，而是持续发布 STOP
                # 安全锁定后忽略按键，持续发送停车/回中值。
                motor_pwm = self.STOP_PWM
                servo_pwm = self.STOP_PWM
                key = ""
            else:
                # 每次读取后清空，避免同一次按键在多个定时周期重复执行。
                key = self._key
                self._key = ""

                if key == "w":
                    self.motor_pwm = min(self.motor_pwm + self.motor_step, self.motor_max)
                elif key == "s":
                    self.motor_pwm = max(self.motor_pwm - self.motor_step, self.motor_min)
                elif key == "j":
                    self.servo_pwm = max(self.servo_pwm - self.step, self.servo_min)
                elif key == "k":
                    self.servo_pwm = min(self.servo_pwm + self.step, self.servo_max)
                elif key == "i":
                    self.servo_pwm = self.STOP_PWM
                elif key == "q":
                    self.motor_pwm = self.STOP_PWM
                    self.servo_pwm = self.STOP_PWM

                motor_pwm = self.motor_pwm
                servo_pwm = self.servo_pwm

        self.publisher.publish(self._make_msg(motor_pwm, servo_pwm))

        # 降低日志量；安全锁定日志由 latch_stop 单独打印。
        if key:
            self.get_logger().info(
                f"发送 -> 电机PWM={motor_pwm:.0f}, 舵机PWM={servo_pwm:.0f}"
            )

    def restore_terminal(self) -> None:
        """通知后台线程退出，并恢复启动前的终端输入设置。"""
        self._stop_event.set()
        try:
            termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.settings)
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[TestCar] = None

    try:
        node = TestCar()

        def handle_signal(sig, _frame) -> None:
            # 先锁定停车并重复发送，再进入 finally 执行清理。
            name = signal.Signals(sig).name
            if node is not None:
                node.latch_stop(f"收到 {name}")
                node.publish_stop_burst()
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)
        if hasattr(signal, "SIGHUP"):
            signal.signal(signal.SIGHUP, handle_signal)

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # 正常退出或异常退出时，再尝试停车并恢复终端。
            node.latch_stop("节点退出")
            node.publish_stop_burst()
            node.restore_terminal()
            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
