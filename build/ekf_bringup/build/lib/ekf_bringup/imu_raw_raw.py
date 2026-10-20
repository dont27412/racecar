#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CH100 -> ROS 2 六轴 IMU 驱动。

固定配置：/dev/imu，460800 baud，100 Hz，AT+MODE=0（六轴）。
文件/节点：imu_raw_raw.py / imu_raw_raw
话题/类型：/imu_raw_raw / sensor_msgs/msg/Imu

启动时识别当前串口波特率，发送 AT 配置并复位，然后开始采集。
配置阶段的数据不发布；采集阶段每个 CRC 正确且六轴数值有限的 0x91
数据帧恰好发布一条消息，不定时重发、不补帧、不积分生成角度或速度。

安装：IMU +X 向车左，+Y 向车尾，+Z 向车顶。
发布坐标：X 向车前，Y 向车左，Z 向车顶；向量变换为 (-Y, X, Z)。
原始消息不设置 frame_id；数值轴向已经在软件中转换为车体 XYZ。如后续由 ekf_bringup
发布为 imu_link，再通过静态 TF 描述 imu_link 到 base_link 的安装位移。
TF，请使用实际安装位移，勿再次应用上述安装旋转。

角速度：deg/s -> rad/s；加速度：g -> m/s^2，保留重力响应。
Imu 没有欧拉角、线速度或独立的设备计时字段；此驱动只发布六轴数据，
orientation_covariance[0] = -1 表示不提供姿态。其余协方差为 0（未知）。

header.stamp：设备开机毫秒计时持续映射到主机 SYSTEM_TIME。驱动保留设备
采样间隔并处理 uint32 回绕，同时用最近约 2 s 的低延迟接收样本持续修正
设备晶振与主机时钟的长期漂移；主机接收时刻会扣除一帧串口理论线传输
时间。重复/异常倒退时间戳不会进入 EKF；连续多帧确认设备重启后重新对齐。
这是软件同步，仍不等同于 IMU/编码器之间的硬件触发同步。

不创建 ROS 2 参数服务，不读取或声明驱动参数；修改配置请编辑下方常量。
启动前关闭占用同一串口的上位机/驱动；运行：python3 imu_raw_raw.py

协议依据：用户提供的《超核电子 CH100 用户手册》，0x91 / AT 指令章节。
ROS API：https://docs.ros2.org/dashing/api/rclpy/api/node.html
日志限频：https://docs.ros.org/en/foxy/Concepts/About-Logging.html
"""

# Python 标准库：
# - math：数值有限性检查、角度转弧度常量
# - re：解析 AT 指令的文本应答
# - struct：按 CH100 二进制协议解包整数/浮点数
# - time：启动等待、超时判断、统计周期
import math
import re
import struct
import time
from collections import deque

# pyserial：访问 /dev/imu 串口。
import serial

# ROS 2 Python 客户端库。
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import Imu


# ============================================================
# 1. 驱动固定配置
# ============================================================
# 本驱动刻意不使用 ROS 2 参数服务，目的是让运行时配置保持唯一、明确。
# 如需修改串口、波特率、输出频率、节点名或话题名，直接修改下面常量。
# CH100 在 Linux 中对应的串口设备。
SERIAL_PORT = "/dev/imu"
# 驱动最终要求 IMU 工作在 460800 baud。
BAUD_RATE = 460800
# 目标输出数据率 ODR：100 Hz，即理想采样周期约 10 ms。
OUTPUT_RATE_HZ = 100
IMU_MODE = 0  # CH100：0 = 六轴，1 = 九轴。
# ROS 2 节点名称。
NODE_NAME = "imu_raw_raw"
# sensor_msgs/msg/Imu 发布话题。
TOPIC_NAME = "/imu_raw_raw"

# 启动阶段并不知道设备当前保存的是哪个波特率，因此逐个尝试。
# 先尝试目标 460800，也兼容常见/出厂配置。
BAUD_CANDIDATES = (BAUD_RATE, 115200, 921600, 9600)
# 单次串口 read() 最长阻塞时间。设置较小值，避免采集循环卡住。
SERIAL_TIMEOUT_S = 0.01
# 发送 AT 指令后收集设备应答的时间窗口。
COMMAND_WAIT_S = 0.3
# AT+RST 后等待设备重新启动的时间。
RESET_WAIT_S = 2.0
# 连续多久没有“有效 IMU 帧”就打印通信警告。
NO_DATA_TIMEOUT_S = 2.0
# 每次最多从串口读取的字节数，防止一次处理过大的缓存。
MAX_READ_BYTES = 4096
# 标准重力加速度，用于 g -> m/s²。
G_TO_MPS2 = 9.80665
# 角度单位换算：deg/s -> rad/s。
DEG_TO_RAD = math.pi / 180.0
# CH100 设备时间戳为 uint32 毫秒计时器。
# 下面两个常量用于安全处理无符号 32 位回绕与“时间倒退”判断。
UINT32_MASK = 0xFFFFFFFF
HALF_UINT32 = 0x80000000
# 时间单位换算常量。
NS_PER_MS = 1_000_000
NS_PER_S = 1_000_000_000

# ============================================================
# 2. CH100 二进制协议常量
# ============================================================
# 每个 CH100 外层数据帧以 0x5A 0xA5 开头。
SYNC = b"\x5A\xA5"
# 外层帧头共 6 字节：同步头/类型/长度/CRC 等信息。
HEADER_SIZE = 6
# 默认 0x91 IMUSOL 数据包的 payload 固定为 76 字节。
IMU_PAYLOAD_SIZE = 76
# 0x91 表示 IMUSOL 数据包。
IMU_TAG = 0x91

# 一帧 0x91 在 8N1 串口上的理论线传输时间。header.stamp 应尽量表示
# “传感器采样时刻”，而不是“最后一个串口字节到达主机的时刻”，因此在
# 主机接收时间上扣除这一固定传输延迟。实际 USB/UART 调度还会有少量抖动，
# 由 DeviceTimestamp 的滑动窗口同步继续抑制。
SERIAL_BITS_PER_BYTE = 10
IMU_FRAME_WIRE_TIME_NS = int(
    (HEADER_SIZE + IMU_PAYLOAD_SIZE) * SERIAL_BITS_PER_BYTE / BAUD_RATE * NS_PER_S
)


# ============================================================
# 3. CH100 数据帧解析器
# ============================================================
class CH100Parser:
    """保留分包缓存，校验 CRC，只接受默认的单个 0x91 数据包。"""

    def __init__(self):
        # 串口 read() 不保证一次就得到完整帧，因此必须保留跨 read 的残余字节。
        self.buffer = bytearray()
        # 累计 CRC 错误数量，便于运行时判断链路质量。
        self.crc_errors = 0
        # 累计出现 NaN / Inf 等非法六轴数值的帧数。
        self.invalid_values = 0

    @staticmethod
    def crc16(data):
        # CH100 手册使用的 CRC16，多项式 0x1021，初始值为 0。
        # CRC 覆盖帧中除 CRC 字段自身以外的内容。
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def feed(self, data):
        """yield (设备毫秒时间戳, Acc XYZ[g], Gyro XYZ[deg/s], Euler RPY[deg])。"""
        # 把本次串口读到的字节追加到缓存，和上一次未完整的数据拼接。
        self.buffer.extend(data)
        while True:
            # 在任意字节流中重新寻找 0x5A 0xA5 帧头。
            sync_pos = self.buffer.find(SYNC)
            if sync_pos < 0:
                # 末尾的 0x5A 可能是尚未收全的帧头。
                self.buffer[:] = b"\x5A" if self.buffer[-1:] == b"\x5A" else b""
                return
            if sync_pos:
                del self.buffer[:sync_pos]
            if len(self.buffer) < HEADER_SIZE:
                return

            # 偏移 2 处为小端 uint16 payload 长度。
            payload_len = struct.unpack_from("<H", self.buffer, 2)[0]
            if payload_len != IMU_PAYLOAD_SIZE:
                del self.buffer[0]
                continue
            frame_len = HEADER_SIZE + payload_len
            if len(self.buffer) < frame_len:
                return

            # 到这里说明缓存中已经存在一整个候选帧。
            frame = bytes(self.buffer[:frame_len])
            # 帧中偏移 4 处保存设备发送过来的 CRC。
            received_crc = struct.unpack_from("<H", frame, 4)[0]
            if received_crc != self.crc16(frame[:4] + frame[6:]):
                self.crc_errors += 1
                del self.buffer[0]
                continue

            # CRC 正确后才一次性移除整帧，后续开始解析 payload。
            del self.buffer[:frame_len]
            payload = frame[HEADER_SIZE:]
            if payload[0] != IMU_TAG:
                continue
            # 下面四个偏移严格对应 CH100 0x91 数据布局：
            #   +8  : uint32 设备开机毫秒时间戳
            #   +12 : float32 Acc X/Y/Z，单位 g
            #   +24 : float32 Gyro X/Y/Z，单位 deg/s
            #   +48 : float32 Euler Roll/Pitch/Yaw，单位 deg
            timestamp_ms = struct.unpack_from("<I", payload, 8)[0]
            acc_g = struct.unpack_from("<fff", payload, 12)
            gyr_dps = struct.unpack_from("<fff", payload, 24)
            eul_deg = struct.unpack_from("<fff", payload, 48)
            # 即使 CRC 正确，也拒绝 NaN / +Inf / -Inf，避免污染后续滤波器。
            if not all(math.isfinite(value) for value in acc_g + gyr_dps):
                self.invalid_values += 1
                continue
            yield timestamp_ms, acc_g, gyr_dps, eul_deg


# ============================================================
# 4. 设备时间戳 -> ROS 系统时间
# ============================================================
class DeviceTimestamp:
    """把 CH100 设备时间映射到主机 SYSTEM_TIME，并持续校正两块时钟的漂移。

    设计目标：
    1. 正常帧/uint32 回绕使用设备时间差，保留 100 Hz 采样间隔；
    2. 所有 ROS 时间戳最终锁定到与编码器相同的主机 SYSTEM_TIME；
    3. 不再只在启动时锚定一次，而是利用最近一段时间的接收时刻持续估计
       "主机时间 - 设备时间" 偏移，避免设备晶振漂移导致 IMU 与编码器逐渐错位；
    4. 接收调度延迟只会让候选时间变晚，因此窗口内取最小偏移，近似选择
       延迟最小的一批样本，再用限速方式平滑跟踪，避免时间戳抖动；
    5. 重复/倒退时间戳不进入正式时间轴；确认设备重启后重新建立锚点。
    """

    RESTART_CONFIRM_FRAMES = 3
    RESTART_MAX_STEP_MS = 3.0 * 1000.0 / OUTPUT_RATE_HZ

    # 使用约 2 秒的低延迟样本估计 host-device offset。
    SYNC_WINDOW_FRAMES = max(20, int(2.0 * OUTPUT_RATE_HZ))
    # 每个 100 Hz 样本最多修正 0.1 ms，即最多约 10 ms/s。
    # 这远大于常见晶振漂移，又不会把主机调度抖动直接写进时间戳。
    MAX_OFFSET_SLEW_NS_PER_FRAME = 100_000
    # 若映射时间与当前低延迟接收估计相差超过 100 ms，说明时钟发生了
    # 大跳变或此前基准已失效，直接重新同步，避免长期使用错误时间轴。
    HARD_RESYNC_THRESHOLD_NS = 100_000_000

    def __init__(self):
        self.last_raw_ms = None
        self.last_unwrapped_ms = None
        self.last_stamp_ns = None
        self.offset_ns = None
        self.offset_candidates = deque(maxlen=self.SYNC_WINDOW_FRAMES)
        self.restart_candidate_count = 0
        self.restart_candidate_last_ms = None

    @staticmethod
    def _batch_age_ms(timestamp_ms, batch_last_ms):
        """计算本帧相对当前 read() 批次末帧的设备时间差。"""
        age_ms = (batch_last_ms - timestamp_ms) & UINT32_MASK
        return 0 if age_ms >= HALF_UINT32 else age_ms

    def _observed_sample_ns(self, timestamp_ms, receive_ns, batch_last_ms):
        """用主机接收时刻估算本帧真实采样时刻。"""
        age_ms = self._batch_age_ms(timestamp_ms, batch_last_ms)
        return receive_ns - age_ms * NS_PER_MS - IMU_FRAME_WIRE_TIME_NS

    def _clear_restart_candidate(self):
        self.restart_candidate_count = 0
        self.restart_candidate_last_ms = None

    def _reset_sync(self, timestamp_ms, receive_ns, batch_last_ms, min_stamp_ns=None):
        """在首帧或确认设备重启后重新建立设备时间到主机时间的映射。"""
        self.last_unwrapped_ms = int(timestamp_ms)
        observed_ns = self._observed_sample_ns(timestamp_ms, receive_ns, batch_last_ms)
        stamp_ns = observed_ns
        if min_stamp_ns is not None and stamp_ns <= min_stamp_ns:
            # 同一 ROS 进程内保证本话题严格单调，避免 EKF 收到倒退时间。
            stamp_ns = min_stamp_ns + 1

        self.offset_ns = stamp_ns - self.last_unwrapped_ms * NS_PER_MS
        self.offset_candidates.clear()
        self.offset_candidates.append(self.offset_ns)
        self.last_raw_ms = timestamp_ms
        self.last_stamp_ns = stamp_ns
        self._clear_restart_candidate()
        return stamp_ns

    def _update_host_sync(self, timestamp_ms, receive_ns, batch_last_ms):
        """更新 host-device offset，并返回当前 unwrapped 设备时间对应的 ROS 时间。"""
        observed_ns = self._observed_sample_ns(timestamp_ms, receive_ns, batch_last_ms)
        candidate_offset_ns = observed_ns - self.last_unwrapped_ms * NS_PER_MS
        self.offset_candidates.append(candidate_offset_ns)

        # 串口/调度抖动主要表现为额外正延迟，因此取窗口最小值更接近
        # 固定传输延迟下的真实采样时刻。
        target_offset_ns = min(self.offset_candidates)
        mapped_ns = self.last_unwrapped_ms * NS_PER_MS + self.offset_ns
        target_mapped_ns = self.last_unwrapped_ms * NS_PER_MS + target_offset_ns

        if abs(target_mapped_ns - mapped_ns) > self.HARD_RESYNC_THRESHOLD_NS:
            # 主机系统时间若发生较大跳变，继续慢慢 slew 会让两个传感器
            # 长时间处于不同时间轴，因此重新锁定。
            self.offset_ns = candidate_offset_ns
            self.offset_candidates.clear()
            self.offset_candidates.append(candidate_offset_ns)
        else:
            error_ns = target_offset_ns - self.offset_ns
            slew_ns = max(
                -self.MAX_OFFSET_SLEW_NS_PER_FRAME,
                min(self.MAX_OFFSET_SLEW_NS_PER_FRAME, error_ns),
            )
            self.offset_ns += int(slew_ns)

        stamp_ns = self.last_unwrapped_ms * NS_PER_MS + self.offset_ns
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            stamp_ns = self.last_stamp_ns + 1
            self.offset_ns = stamp_ns - self.last_unwrapped_ms * NS_PER_MS
        return stamp_ns

    def convert(self, timestamp_ms, receive_ns, batch_last_ms):
        """返回 (stamp_ns, restarted, delta_ms)。

        stamp_ns 为 None 表示该设备时间戳不能安全送入 EKF；重复时间戳也会
        丢弃，以保证每条 IMU 测量都有严格递增的采样时刻。
        """
        if self.last_raw_ms is None:
            return self._reset_sync(timestamp_ms, receive_ns, batch_last_ms), False, None

        delta_ms = (timestamp_ms - self.last_raw_ms) & UINT32_MASK

        if delta_ms < HALF_UINT32:
            self._clear_restart_candidate()

            if delta_ms == 0:
                # 同一采样时刻重复发布会让 EKF 在零 dt 上重复更新，直接丢弃。
                return None, False, 0

            self.last_unwrapped_ms += int(delta_ms)
            stamp_ns = self._update_host_sync(timestamp_ms, receive_ns, batch_last_ms)
            self.last_raw_ms = timestamp_ms
            self.last_stamp_ns = stamp_ns
            return stamp_ns, False, delta_ms

        # 当前 timestamp 比上一条已接受值小。先作为重启候选，不破坏正式时间轴。
        if self.restart_candidate_last_ms is None:
            self.restart_candidate_count = 1
            self.restart_candidate_last_ms = timestamp_ms
            return None, False, None

        candidate_delta_ms = (timestamp_ms - self.restart_candidate_last_ms) & UINT32_MASK
        if 0 < candidate_delta_ms <= self.RESTART_MAX_STEP_MS:
            self.restart_candidate_count += 1
        else:
            self.restart_candidate_count = 1
        self.restart_candidate_last_ms = timestamp_ms

        if self.restart_candidate_count < self.RESTART_CONFIRM_FRAMES:
            return None, False, None

        # 确认设备重启。新设备时间轴重新对齐当前主机时间，但仍保持 ROS
        # header.stamp 严格递增，避免 EKF 看到时间倒退。
        previous_stamp_ns = self.last_stamp_ns
        stamp_ns = self._reset_sync(
            timestamp_ms, receive_ns, batch_last_ms, min_stamp_ns=previous_stamp_ns
        )
        return stamp_ns, True, None


# ============================================================
# 5. 坐标系转换 + SI 单位转换
# ============================================================
def to_body_si(acc_g, gyr_dps):
    """IMU 左/后/上 -> 车体前/左/上，同时换算 SI 单位。"""
    # 传感器实际安装方向：IMU +X=车左，+Y=车尾，+Z=车顶。
    # ROS/车体希望使用：+X=车前，+Y=车左，+Z=车顶。
    # 因此向量映射为 Body = (-IMU_Y, IMU_X, IMU_Z)。
    # 同时加速度从 g 转为 m/s²。
    acc = (-acc_g[1] * G_TO_MPS2, acc_g[0] * G_TO_MPS2, acc_g[2] * G_TO_MPS2)
    # 角速度做相同坐标变换，并从 deg/s 转为 rad/s。
    gyr = (-gyr_dps[1] * DEG_TO_RAD, gyr_dps[0] * DEG_TO_RAD, gyr_dps[2] * DEG_TO_RAD)
    return acc, gyr


# ============================================================
# 6. ROS 2 节点
# ============================================================
class IMUNode(Node):
    def __init__(self):
        super().__init__(
            NODE_NAME,
            start_parameter_services=False,
            use_global_arguments=False,
            automatically_declare_parameters_from_overrides=False,
        )
        self.info_time_s = 1.0  # INFO/WARN/ERROR 间隔，单位：秒。
        # QoS depth=10：保存最多 10 条尚未被订阅者处理的消息。
        self.publisher = self.create_publisher(Imu, TOPIC_NAME, 10)
        # 明确使用实机系统时间，不依赖 /clock 或 use_sim_time 参数。
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.ser = None
        # 协议解析器和设备时间转换器都在节点生命周期内持续保存状态。
        self.parser = CH100Parser()
        self.device_clock = DeviceTimestamp()
        # 以下变量仅用于日志统计，不影响发布的数据内容。
        self.total_frames = 0
        self.window_frames = 0
        self.window_first_ms = None
        self.window_last_ms = None
        self.window_start_s = time.monotonic()
        self.last_data_s = self.window_start_s
        self.unconfirmed_commands = []

    def _read_for(self, duration_s):
        """仅用于启动配置，收集应答；采集循环不用此等待窗口。"""
        end_s = time.monotonic() + duration_s
        data = bytearray()
        while time.monotonic() < end_s:
            if not rclpy.ok():
                raise KeyboardInterrupt
            data.extend(self.ser.read(min(self.ser.in_waiting or 1, MAX_READ_BYTES)))
        return bytes(data)

    def _write_at(self, command):
        # CH100 AT 指令以 CRLF 结尾。
        self.ser.write((command + "\r\n").encode("ascii"))
        self.ser.flush()

    def _send_at(self, command):
        """检查显式错误；手册未规定统一 ACK，缺少 ACK 时明确记录。"""
        # 清掉连续二进制输出/旧应答，尽量让本次 response 只包含当前命令结果。
        self.ser.reset_input_buffer()
        self._write_at(command)
        response = self._read_for(COMMAND_WAIT_S)
        text = response.decode("ascii", errors="ignore")
        if re.search(r"\b(?:ERROR|ERR|FAIL|FAILED|INVALID|UNSUPPORTED)\b|NOT\s+SUPPORT", text, re.I):
            raise RuntimeError("设备拒绝 {}，应答={!r}".format(command, text[-300:]))
        if not re.search(r"\b(?:OK|SUCCESS)\b", text, re.I):
            self.unconfirmed_commands.append(command)
        return response

    def configure(self):
        # 识别设备当前波特率，并强制写入本驱动要求的配置。
        # exclusive=True 可以避免其他进程同时打开同一串口（平台支持时）。
        self.ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=SERIAL_TIMEOUT_S,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            exclusive=True,
        )
        # 第一步：尝试多个波特率，并通过 AT+INFO 或有效 0x91 数据识别 CH100。
        current_baud = None
        for baud in BAUD_CANDIDATES:
            self.ser.baudrate = baud
            self.ser.reset_input_buffer()
            # 清除设备可能保留的半条指令，再查询；不会写入设备配置。
            # 先发送一个空行，尽量结束设备可能保留的半条 AT 指令。
            self.ser.write(b"\r\n")
            self._write_at("AT+INFO")
            response = self._read_for(0.5)
            if list(CH100Parser().feed(response)) or b"CH100" in response.upper():
                current_baud = baud
                break
        if current_baud is None:
            raise RuntimeError(
                "未识别到 CH100：检查 /dev/imu、供电、TX/RX、串口占用及默认 0x91 输出。"
            )
        self.get_logger().info(
            "识别串口 {} @ {} baud，开始配置 {} baud / {} Hz / {}轴模式".format(
                SERIAL_PORT,
                current_baud,
                BAUD_RATE,
                OUTPUT_RATE_HZ,
                6 if IMU_MODE == 0 else 9,
            ),
            throttle_duration_sec=self.info_time_s,
        )

        # ODR、BAUD、MODE 按手册掉电保存、复位生效；复位前仍用旧波特率。
        # 第二步：先关闭连续数据输出，避免二进制数据干扰 AT 命令通信。
        self._send_at("AT+EOUT=0")
        self.ser.reset_input_buffer()
        if list(CH100Parser().feed(self._read_for(0.25))):
            raise RuntimeError("AT+EOUT=0 后设备仍在输出：请检查主机 TX 到 IMU RX 的连接。")
        # 第三步：写入目标波特率、输出频率、六轴模式和单位坐标矩阵。
        # URFR 设置为单位矩阵，保证设备内部不预先旋转数据；安装方向只在
        # to_body_si() 中统一转换一次，避免“双重旋转”。
        for command in (
            "AT+BAUD={}".format(BAUD_RATE),
            "AT+ODR={}".format(OUTPUT_RATE_HZ),
            "AT+MODE={}".format(IMU_MODE),
            # 恢复设备原始轴定义，安装旋转只在 to_body_si() 中执行一次。
            "AT+URFR=1,0,0,0,1,0,0,0,1",
        ):
            self._send_at(command)

        # 第四步：复位使掉电保存配置生效，然后切换主机串口到目标波特率。
        self._write_at("AT+RST")
        self.ser.baudrate = BAUD_RATE
        self._read_for(RESET_WAIT_S)
        # 第五步：重新打开连续数据输出，正式进入采集阶段。
        self._send_at("AT+EOUT=1")
        self.ser.reset_input_buffer()

        if self.unconfirmed_commands:
            self.get_logger().warning(
                "以下指令已发送但未收到可识别的 OK/SUCCESS：{}。"
                "将按实际数据检查输出频率；0x91 六轴数据本身不能证明 MODE/URFR 配置已生效。".format(
                    ", ".join(self.unconfirmed_commands)
                ),
                throttle_duration_sec=self.info_time_s,
            )
        self.get_logger().info(
            "配置指令发送完毕，等待有效 0x91 数据；topic={}，raw 消息不设置 frame_id，车体 XYZ=(-IMU_Y, IMU_X, IMU_Z)".format(
                TOPIC_NAME
            ),
            throttle_duration_sec=self.info_time_s,
        )
        self.window_start_s = self.last_data_s = time.monotonic()

    def publish_imu(self, timestamp_ms, acc_g, gyr_dps, eul_deg, receive_ns, batch_last_ms):
        # 把一帧有效 CH100 六轴数据转换并发布成 sensor_msgs/msg/Imu。
        stamp_ns, restarted, delta_ms = self.device_clock.convert(
            timestamp_ms, receive_ns, batch_last_ms
        )

        # 单帧时间戳倒退先视为异常帧，不让它修改正式时间基准。
        # 如果后续连续多帧都在新的较小时间轴上递增，DeviceTimestamp
        # 会自动确认设备重启，并从确认帧开始恢复发布。
        if stamp_ns is None:
            if delta_ms == 0:
                text = "检测到重复 IMU 设备时间戳 {} ms，本帧丢弃，避免 EKF 零 dt 重复更新。".format(
                    timestamp_ms
                )
            else:
                text = "检测到异常/倒退 IMU 设备时间戳 {} ms；暂不判定为重启，本帧丢弃。".format(
                    timestamp_ms
                )
            self.get_logger().warning(
                text,
                throttle_duration_sec=self.info_time_s,
            )
            return

        if restarted:
            self.get_logger().warning(
                "连续时间戳确认 IMU 已重启，已重新对齐 ROS 时间。",
                throttle_duration_sec=self.info_time_s,
            )
            self.window_frames = 0
            self.window_first_ms = None
            self.window_last_ms = None
            self.window_start_s = time.monotonic()
        elif delta_ms is not None and (
            delta_ms == 0
            or delta_ms > 1.5 * 1000.0 / OUTPUT_RATE_HZ
        ):
            self.get_logger().warning(
                "设备时间戳间隔={} ms，目标约 {:.1f} ms；"
                "重复时间戳或较大间隔需要检查，未补帧。".format(
                    delta_ms, 1000.0 / OUTPUT_RATE_HZ
                ),
                throttle_duration_sec=self.info_time_s,
            )

        # 先完成坐标系与物理单位转换。
        acc, gyr = to_body_si(acc_g, gyr_dps)
        # 构造 ROS 标准 IMU 消息。
        msg = Imu()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, NS_PER_S)
        # 本驱动不发布姿态。w=1 只是给四元数字段一个合法默认值；
        # orientation_covariance[0] = -1 才是 ROS 约定的“姿态不可用”标志。
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0
        # 填入已经转换到车体坐标系、SI 单位的加速度与角速度。
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyr
        # 每个 CRC 正确且六轴数值有效的 0x91 帧只发布一次，不补帧、不重发。
        self.publisher.publish(msg)

        self.total_frames += 1
        self.window_frames += 1
        if self.window_first_ms is None:
            self.window_first_ms = timestamp_ms
        self.window_last_ms = timestamp_ms
        self.last_data_s = time.monotonic()
        self._log_statistics(acc, gyr, eul_deg, timestamp_ms, stamp_ns)

    def _log_statistics(self, acc, gyr, eul_deg, timestamp_ms, stamp_ns):
        # 约每秒输出一次接收率、设备时间帧率、当前六轴值和错误计数。
        now_s = time.monotonic()
        elapsed_s = now_s - self.window_start_s
        if elapsed_s < self.info_time_s:
            return
        # host_hz：按主机实际运行时间统计的接收/发布频率。
        host_hz = self.window_frames / elapsed_s
        device_hz = None
        # device_hz：使用设备自身 timestamp 计算，更接近真实采样频率。
        device_ms = (self.window_last_ms - self.window_first_ms) & UINT32_MASK
        if self.window_frames >= 2 and 0 < device_ms < HALF_UINT32:
            device_hz = (self.window_frames - 1) * 1000.0 / device_ms
        device_text = "N/A" if device_hz is None else "{:.1f} Hz".format(device_hz)
        self.get_logger().info(
            "接收/发布帧率={:.1f} Hz | 设备时间有效帧率={} | 本周期帧数={} | 总帧数={} | "
            "设备时间戳={} ms | ROS时间戳={}.{:09d} | "
            "Accel_X={:.6f}, Accel_Y={:.6f}, Accel_Z={:.6f} m/s² | "
            "Gyro_X={:.6f}, Gyro_Y={:.6f}, Gyro_Z={:.6f} rad/s | "
            "Euler(Roll/Pitch/Yaw)=({:.3f}, {:.3f}, {:.3f}) deg | "
            "横滚角={:.3f}° | 俯仰角={:.3f}° | 偏航角={:.3f}° | CRC错误={}".format(
                host_hz, device_text, self.window_frames, self.total_frames,
                timestamp_ms, stamp_ns // NS_PER_S, stamp_ns % NS_PER_S,
                *acc, *gyr, *eul_deg, *eul_deg, self.parser.crc_errors
            ),
            throttle_duration_sec=self.info_time_s,
        )
        if device_hz is not None and abs(device_hz - OUTPUT_RATE_HZ) > OUTPUT_RATE_HZ * 0.05:
            self.get_logger().warning(
                "实测有效帧率 {:.1f} Hz 偏离目标 {} Hz；检查 ODR 配置、CRC错误和时间戳间隔。".format(
                    device_hz, OUTPUT_RATE_HZ
                ),
                throttle_duration_sec=self.info_time_s,
            )
        self.window_start_s = now_s
        self.window_frames = 0
        self.window_first_ms = None
        self.window_last_ms = None

    def run(self):
        # 完成配置后进入串口采集主循环，直到 ROS 关闭或发生异常。
        self.configure()
        while rclpy.ok():
            # 如果系统缓冲区已有数据就尽量一次读走，否则至少读 1 字节等待新数据。
            chunk = self.ser.read(min(self.ser.in_waiting or 1, MAX_READ_BYTES))
            # 本次 read() 完成时的主机系统时间，只用于设备时间首次/重启后的对齐。
            receive_ns = self.system_clock.now().nanoseconds
            crc_before, invalid_before = self.parser.crc_errors, self.parser.invalid_values
            # parser.feed() 可能从一次 read() 中解析出 0 帧、1 帧或多帧。
            frames = list(self.parser.feed(chunk))
            # 以本批末帧辅助首次时间对齐，避免批量到达使前几帧获得未来时间。
            for timestamp_ms, acc_g, gyr_dps, eul_deg in frames:
                self.publish_imu(timestamp_ms, acc_g, gyr_dps, eul_deg, receive_ns, frames[-1][0])
            if self.parser.crc_errors > crc_before or self.parser.invalid_values > invalid_before:
                self.get_logger().warning(
                    "已丢弃损坏帧：累计 CRC错误={}，非有限数值帧={}。".format(
                        self.parser.crc_errors, self.parser.invalid_values
                    ),
                    throttle_duration_sec=self.info_time_s,
                )
            if time.monotonic() - self.last_data_s > NO_DATA_TIMEOUT_S:
                self.get_logger().warning(
                    "超过 2 秒未收到有效 IMU 帧：检查连接、460800 波特率、EOUT 和默认 0x91 协议。",
                    throttle_duration_sec=self.info_time_s,
                )

    def close(self):
        # 安全关闭串口。
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def log_error(self, exc):
        self.get_logger().error(
            "IMU 驱动停止：{}".format(exc),
            throttle_duration_sec=self.info_time_s,
        )


# ============================================================
# 7. 程序入口与资源清理
# ============================================================
def main(args=None):
    # 初始化 ROS 2 context。
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        # 创建节点后，run() 会一直阻塞在串口采集循环中。
        node = IMUNode()
        # 只有串口读取和直接发布，无订阅/定时器/参数服务，无需 spin()。
        node.run()
    except KeyboardInterrupt:
        # Ctrl+C 属于正常退出，不作为错误处理。
        pass
    except (serial.SerialException, OSError, RuntimeError) as exc:
        # Ctrl-C 导致 context 关闭时不再调用 ROS 日志。
        if rclpy.ok():
            if node is not None:
                node.log_error(exc)
            else:
                raise
            exit_code = 1
    finally:
        # 无论正常退出还是异常退出，都关闭串口、销毁节点并关闭 ROS。
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
