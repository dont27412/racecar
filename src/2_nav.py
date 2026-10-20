#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import signal
import os
import subprocess
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import Buffer, TransformListener, TransformException
from robot_localization.srv import SetPose


# ============================================================
# Nav2DControl：2D Pure Pursuit / Ackermann 导航控制
# 功能：
# 1. 使用 /odom_combined 的位姿进行轨迹控制；
# 2. 将赛道原始点整体平移，使车辆启动位置对应局部 (0, 0)；
# 3. 按顺序跟踪目标点位，前视点可跨越后续路径段，不再限制在当前线段；
# 4. 使用以后轴中心地面投影为参考点的 2D Pure Pursuit / Ackermann 运动学控制；
# 5. line_v 直接作为电机 PWM 发布；PP 输出前轮等效转角，再映射为舵机 PWM；
# 6. 最后回到起点后电机 PWM 和舵机 PWM 都回到 1500。
# ============================================================
class Nav2DControl(Node):
    def __init__(self):
        super().__init__('nav_2d_control')

        # ============================================================
        # 1. ROS2 通信基础配置
        # QoS 配置
        # BEST_EFFORT：尽力而为，适合实时控制/TF 这类允许偶发丢包的数据；
        # KEEP_LAST + depth=1：只保留最新一帧数据，避免旧控制信息堆积。
        # ============================================================
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 日志限频时间。
        # 使用 ROS2 logger 自带 throttle，避免50 Hz 控制循环刷屏。
        self.info_time_s = 1.0  # INFO 间隔，单位：秒

        # ============================================================
        # 2. 控制参数 / 调参区

        self.declare_parameter('line_v', 1680.0)       # 电机 PWM 默认值；1500 附近为停止，1650 为当前前进速度
        self.declare_parameter('line_kp', 25.0)        # 22.2
        self.declare_parameter('lookahead_cm', 155.0)   # PP 前视距离，单位 cm

        # 读取 ROS2 参数服务器中的最终值（可能是默认值，也可能被 launch/命令行覆盖）。
        self.line_v = float(self.get_parameter('line_v').value)
        self.line_kp = float(self.get_parameter('line_kp').value)
        self.lookahead_cm = float(self.get_parameter('lookahead_cm').value)

        # 起步电机 PWM 缓升：导航开始时从 1600 起步，1 秒内线性过渡到 line_v。
        self.start_motor_pwm = 1650.0
        self.motor_ramp_duration_s = 0.5
        self.motor_ramp_start_time = None

        # Ackermann 运动学固定参数。
        # 参考点：后轴中心地面投影 akm_rear_axle_footprint。
        self.wheelbase_cm = 34.0                 # 轴距 34 cm；影响“曲率 -> 前轮转角”的换算
        self.center_min_turn_radius_cm = 125.0   # 车体中心最小转弯半径 125 cm；用于限制最大允许转角

        # 实测 1.25 m 为车体几何中心转弯半径。
        # 前后轮关于车体中心对称，换算到后轴中心后计算最大等效前轮转角。
        # 假设车体几何中心位于前后轴中间，因此几何中心到后轴中心距离 = 轴距/2。
        center_to_rear_cm = self.wheelbase_cm * 0.5
        rear_axle_min_turn_radius_cm = math.sqrt(max(
            self.center_min_turn_radius_cm ** 2 - center_to_rear_cm ** 2,
            1.0,
        ))
        # 根据自行车模型 delta = atan(L / R) 计算最大等效前轮转角。
        # 后续 PP 算出的 steering_deg 会被限制在 ±max_steering_deg 内，避免要求车辆做不到的急转。
        self.max_steering_deg = math.degrees(math.atan2(
            self.wheelbase_cm,
            rear_axle_min_turn_radius_cm,
        ))

        # 起步阶段：SLAM/EKF 复位成功后，先保持舵机中位直行这么远，再进入 PP。
        # 作用：让车辆从起点稳定起步，减少刚启动时立即打舵。
        self.start_straight_cm = 10.0  # [cm]

        # 切点 / 停车判定。
        self.stop_tolerance_cm = 40.0              # [cm] 终点停车容差；只用于最后一个点
        self.pass_tolerance_cm = 40.0              # [cm] 普通中间点到点容差；进入后直接切下一点
        self.point_switch_lateral_distance = 50.0  # [cm] 越过目标门线时允许的最大横向偏差

        # PP 小转角死区：|steering| <= 0.5° 时直接输出 0°。
        # 主要抑制直线段因定位噪声造成的舵机来回微抖。
        self.dead_zone_deg = 0.5  # [deg]              

        # 舵机 PWM 标定值。当前映射方向：左转 steering>0 -> PWM 低于 1500；右转相反。
        self.servo_mid = 1500.0  # [PWM] 舵机机械中位/直行值
        self.servo_min = 1100.0  # [PWM] 允许发布的最小舵机值，防止过度左打
        self.servo_max = 1900.0  # [PWM] 允许发布的最大舵机值，防止过度右打

        # 舵机 PWM 变化率限制：50 Hz 控制下，相邻两周期最多变化 20 PWM。
        # 用于避免即使目标转角合法，舵机指令仍在单个周期内突然大跳变。
        self.max_angular_z_change = 50.0  # [PWM / control cycle]
        self.last_angular_z = self.servo_mid

        # ============================================================
        # 3. 路径点与局部坐标，单位 cm
        # 原始起点：(11.23, 0.75) m
        # 换成 cm 并四舍五入后：(1123, 75)
        # 所有点减去这个起点，所以小车启动后的起点为 (0, 0)
        # ============================================================
        # 局部坐标原点在原地图中的位置 (x, y)，单位 cm。
        # local = map - origin，因此地图起点 (1123,75) 会变成局部 (0,0)。
        self.origin_point_cm = (1123, 75)

        # 原始轨迹点。
        # 注意：这些坐标仍然是赛道原始地图坐标，并没有在这里直接改成局部坐标。
        # 后面统一减去 self.origin_point_cm 得到 local_points_cm。
        # 轨迹表首项为起点，末项重复起点作为终点。
        self.map_points_cm = [
            (1123, 75),   # S     | 起点/终点 
            (1405, 75),   # K10   | 特殊点
            (1546, 134),  # R1    | 右侧下圆弧 1/4 点
            (1605, 275),  # K09   | 特殊点
            (1546, 416),  # R2    | 右侧上圆弧 3/4 点
            (1405, 475),  # K08   | 特殊点
            (1264, 417), 
            (1123, 358),  # 折线拐点
            (982, 417), 
            (840, 475),   
            (699, 416),  
            (558, 358),   
            (416, 416),  
            (275, 475),   
            (150, 475), 
            # (75, 475),    # 直角中心点
            (75, 400),   
            (75, 275),  
            (134, 134),   # Larc  | 左下圆弧 1/2 点
            # (275, 75),  # 左圆弧下
            (388, 75), 
            (671, 75),    # A点
            (1123, 75),   # S_end | 终点（重复起点）
        ]
        self.map_points_cm += self.map_points_cm[1:]  # 再接一圈：第1圈回起点后不停，连续跑第2圈

        # 坐标整体平移。
        self.local_points_cm = []
        for x, y in self.map_points_cm:
            local_x = x - self.origin_point_cm[0]
            local_y = y - self.origin_point_cm[1]
            self.local_points_cm.append((local_x, local_y))

        # ============================================================
        # 4. 导航运行状态
        # ============================================================
        # 第 0 个点是起点 (0,0)，车辆已位于该点，因此从第 1 个点开始跟踪。
        self.target_index = 1  # 当前离散目标点下标；0 是已经位于其上的起点，所以从 1 开始

        # START_STRAIGHT：起步舵机中位直行；
        # NAVIGATING：Pure Pursuit 正常跟踪；
        # FINISHED：全部路径完成，持续停车。
        self.state = "NAVIGATING"  # 运行状态：START_STRAIGHT / NAVIGATING / FINISHED

        self.start_straight_origin = None

        # PP 转角滤波状态。
        self.last_steering_deg = 0.0  # 上一次滤波后的转角，供一阶低通滤波使用
        self.steering_window = deque(maxlen=3)  # 最近 3 次原始转角；取中值可抑制单次异常跳变

        # ============================================================
        # 5. TF 与启动复位状态
        # 启动阶段读取全局位姿：map -> akm_rear_axle_footprint
        # ============================================================
        # TF Buffer 用于缓存坐标变换；TransformListener 会自动监听 /tf 和 /tf_static。
        # lookup_transform('map', 'akm_rear_axle_footprint', Time())
        # 返回的就是后轴中心在 map 坐标系下的全局位姿。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_frame = 'map'
        self.base_frame = 'akm_rear_axle_footprint'

        # ============================================================
        # 每次启动本导航脚本时，只在本文件内部完成“新一圈复位”。
        # 其它源码和 all.launch.py 都不用修改：
        #   1. 先停止 all.launch.py 当前正在运行的旧 slam_toolbox；
        #   2. 调用 robot_localization /set_pose，把 EKF 的 x/y/yaw 归零；
        #   3. 等待 3.0 s，让 laser_deskew 中 2.0 s 的旧 odom 历史自然淘汰；
        #   4. 本节点重新启动一个全新的 slam_toolbox；
        #   5. 用全新的 TF Buffer 等待新的 map -> base TF，确认后开始导航。
        #
        # racecar_driver、雷达、IMU、编码器、ekf_bringup、Madgwick、EKF、deskew
        # 全部保持运行，不需要手工重启。
        # 复位完成以前，本节点只发布电机 PWM=1500 / 舵机 PWM=1500 的停车指令。
        # ============================================================
        self.ekf_set_pose_client = self.create_client(SetPose, '/set_pose')

        self.startup_reset_done = False
        self.startup_reset_phase = 'STOP_OLD_SLAM'
        self.startup_reset_future = None
        self.startup_reset_deadline = 0.0
        self.startup_reset_retry_after = 0.0
        self.old_slam_stop_stage = 0

        # 由本导航脚本启动的新 slam_toolbox 的 ros2 launch 进程。
        # start_new_session=True 后可以只关闭这一组进程，不影响其它常驻节点。
        self.slam_process = None

        # laser_deskew/config/deskew.yaml 对应源码默认 odom_history_seconds=2.0。
        # 额外留 1.0 s 裕量，确保新 SLAM 启动后不会消费跨越 EKF 归零瞬间的扫描。
        # laser_deskew 默认保留约 2 s 的 odom 历史，这里等待 3 s，给 1 s 额外裕量。
        # 调得过短可能让 EKF 归零前后的旧/新里程计数据混在一起；通常无需频繁修改。
        self.deskew_flush_seconds = 3.0  # [s]

        # ============================================================
        # 6. ROS2 发布、订阅与控制定时器
        # ============================================================
        # racecar_driver 对 /cmd_vel 的约定：
        #   linear.x  -> 电机 PWM
        #   angular.z -> 舵机 PWM
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # /odom_combined 作为正常导航控制的位姿来源。
        # TF 监听同时用于启动阶段确认 map -> 后轴参考点，以及必要时将 odom child 换算到后轴参考点。
        self.odom_pose = None
        self.odom_rear_axle_offset_child_frame = None
        self.odom_rear_axle_offset_xy = None
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom_combined',
            self.odom_callback,
            qos_profile,
        )

        # 控制周期 0.02 s = 20 ms，对应 50 Hz。
        # 周期更短：控制更新更快但 CPU/通信负担更高；周期更长：响应会变慢。
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            "nav_2d_control 已启动：自动停止旧 SLAM、EKF 归零并启动新 SLAM，完成后开始导航",
            throttle_duration_sec=self.info_time_s,
        )
        self.get_logger().info(
            f"当前目标点 index={self.target_index}, "
            f"地图坐标={self.map_points_cm[self.target_index]}, "
            f"局部坐标={self.local_points_cm[self.target_index]}",
            throttle_duration_sec=self.info_time_s,
        )

    # ============================================================
    # 启动复位与 SLAM 生命周期管理
    # 只修改本 nav 文件；all.launch.py 和其它节点保持原样。
    # ============================================================
    def _find_slam_pids(self):
        """查找当前 async_slam_toolbox_node 的 PID。"""
        try:
            result = subprocess.run(
                ['pgrep', '-f', '[a]sync_slam_toolbox_node'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=0.5,
                check=False,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"查找旧 slam_toolbox 进程失败: {exc}",
                throttle_duration_sec=self.info_time_s,
            )
            return []

        pids = []
        for line in result.stdout.splitlines():
            try:
                pid = int(line.strip())
            except ValueError:
                continue
            if pid > 1 and pid != os.getpid():
                pids.append(pid)
        return pids

    def _signal_slam_pids(self, sig):
        """只向 slam_toolbox 节点发信号，不碰雷达/驱动/EKF 等其它节点。"""
        for pid in self._find_slam_pids():
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError as exc:
                self.get_logger().error(f"无权限停止 slam_toolbox pid={pid}: {exc}")

    def _request_set_pose_zero(self):
        """调用 robot_localization /set_pose，把本圈起点定义为 odom=(0,0,0)。"""
        if not self.ekf_set_pose_client.service_is_ready():
            return False

        request = SetPose.Request()
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.header.frame_id = 'odom'
        request.pose.pose.pose.position.x = 0.0
        request.pose.pose.pose.position.y = 0.0
        request.pose.pose.pose.position.z = 0.0
        request.pose.pose.pose.orientation.x = 0.0
        request.pose.pose.pose.orientation.y = 0.0
        request.pose.pose.pose.orientation.z = 0.0
        request.pose.pose.pose.orientation.w = 1.0
        request.pose.pose.covariance[0] = 1e-9
        request.pose.pose.covariance[7] = 1e-9
        request.pose.pose.covariance[35] = 1e-9

        self.get_logger().warning("正在通过 /set_pose 将 EKF 起点归零")
        self.startup_reset_future = self.ekf_set_pose_client.call_async(request)
        self.startup_reset_phase = 'WAIT_SET_POSE'
        return True

    def _start_fresh_slam(self):
        """启动一个新的 slam_toolbox；配置仍使用工程原来的 bringup launch。"""
        # 防止异常重试时遗留由本节点创建的 ros2 launch 进程。
        self.stop_owned_slam()

        try:
            self.get_logger().warning("正在启动全新的 slam_toolbox...")
            self.slam_process = subprocess.Popen(
                [
                    'ros2', 'launch',
                    'slam_toolbox_bringup',
                    'slam_toolbox_bringup.launch.py',
                    'use_sim_time:=false',
                ],
                start_new_session=True,
            )
        except Exception as exc:
            self.get_logger().error(f"启动 slam_toolbox 失败，将重试: {exc}")
            self.slam_process = None
            return False

        # 关键：换一个全新的 TF Buffer。
        # 这样绝不会误读启动本 nav 之前旧 slam_toolbox 留下的 map->odom 动态 TF。
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.startup_reset_deadline = time.monotonic() + 12.0
        self.startup_reset_phase = 'WAIT_FRESH_SLAM_TF'
        return True

    def stop_owned_slam(self):
        """关闭由本 nav 脚本自己启动的 SLAM 进程组，不影响其它常驻进程。"""
        proc = self.slam_process
        if proc is None:
            return

        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                try:
                    proc.wait(timeout=1.5)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    try:
                        proc.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            except Exception as exc:
                self.get_logger().warning(f"关闭本轮 slam_toolbox 时出现异常: {exc}")

        self.slam_process = None

    def startup_reset_step(self):
        """非阻塞启动复位状态机；由 50 Hz control_loop 周期调用。"""
        if self.startup_reset_done:
            return True

        # 复位期间绝不输出前进命令。
        self.publish_stop()
        now_s = time.monotonic()

        if now_s < self.startup_reset_retry_after:
            return False

        # ------------------------------------------------------------
        # 1) 停掉 all.launch.py 已经启动的旧 slam_toolbox。
        # all.launch.py 中该 Node 没有 respawn，因此杀掉后不会自己重启。
        # ------------------------------------------------------------
        if self.startup_reset_phase == 'STOP_OLD_SLAM':
            pids = self._find_slam_pids()
            if not pids:
                self.get_logger().info("旧 slam_toolbox 已停止")
                self.old_slam_stop_stage = 0
                self.startup_reset_phase = 'WAIT_EKF_SERVICE'
                return False

            if self.old_slam_stop_stage == 0:
                self.get_logger().warning(f"停止旧 slam_toolbox，pid={pids}")
                self._signal_slam_pids(signal.SIGINT)
                self.old_slam_stop_stage = 1
                self.startup_reset_deadline = now_s + 1.0
                return False

            if now_s < self.startup_reset_deadline:
                return False

            if self.old_slam_stop_stage == 1:
                self.get_logger().warning("旧 slam_toolbox 未退出，发送 SIGTERM")
                self._signal_slam_pids(signal.SIGTERM)
                self.old_slam_stop_stage = 2
                self.startup_reset_deadline = now_s + 1.0
                return False

            self.get_logger().warning("旧 slam_toolbox 仍未退出，发送 SIGKILL")
            self._signal_slam_pids(signal.SIGKILL)
            self.old_slam_stop_stage = 3
            self.startup_reset_deadline = now_s + 0.3
            return False

        # ------------------------------------------------------------
        # 2) EKF 归零。
        # ------------------------------------------------------------
        if self.startup_reset_phase == 'WAIT_EKF_SERVICE':
            if not self._request_set_pose_zero():
                self.get_logger().warning(
                    "等待 robot_localization /set_pose 服务...",
                    throttle_duration_sec=self.info_time_s,
                )
            return False

        if self.startup_reset_phase == 'WAIT_SET_POSE':
            if self.startup_reset_future is None or not self.startup_reset_future.done():
                return False
            try:
                self.startup_reset_future.result()
            except Exception as exc:
                self.get_logger().error(f"EKF /set_pose 失败，将重试: {exc}")
                self.startup_reset_phase = 'WAIT_EKF_SERVICE'
                self.startup_reset_future = None
                self.startup_reset_retry_after = now_s + 0.5
                return False

            self.startup_reset_future = None
            self.startup_reset_deadline = now_s + self.deskew_flush_seconds
            self.startup_reset_phase = 'WAIT_DESKEW_FLUSH'
            self.get_logger().info(
                f"EKF 已归零；等待 {self.deskew_flush_seconds:.1f}s "
                "让 laser_deskew 的旧 odom 历史自然淘汰"
            )
            return False

        # ------------------------------------------------------------
        # 3) 不重启 deskew，只等待它的 2 秒历史缓存被新的零点 odom 覆盖。
        # ------------------------------------------------------------
        if self.startup_reset_phase == 'WAIT_DESKEW_FLUSH':
            if now_s < self.startup_reset_deadline:
                return False
            self.startup_reset_phase = 'START_FRESH_SLAM'
            return False

        # ------------------------------------------------------------
        # 4) 新建 SLAM 进程。
        # ------------------------------------------------------------
        if self.startup_reset_phase == 'START_FRESH_SLAM':
            if not self._start_fresh_slam():
                self.startup_reset_retry_after = now_s + 1.0
                return False
            return False

        # ------------------------------------------------------------
        # 5) 等待新 SLAM 真正建立 map -> base。
        # 使用刚创建的空 TF Buffer，不可能把旧 SLAM 的 TF 当成成功。
        # ------------------------------------------------------------
        if self.startup_reset_phase == 'WAIT_FRESH_SLAM_TF':
            if self.slam_process is not None and self.slam_process.poll() is not None:
                rc = self.slam_process.returncode
                self.get_logger().error(f"新 slam_toolbox 提前退出，returncode={rc}，准备重试")
                self.slam_process = None
                self.startup_reset_phase = 'START_FRESH_SLAM'
                self.startup_reset_retry_after = now_s + 1.0
                return False

            pose = self.get_global_pose()
            if pose is None:
                if now_s >= self.startup_reset_deadline:
                    self.get_logger().error(
                        "启动新 SLAM 后 12 秒仍未得到 map -> base TF；将重启 SLAM 再试"
                    )
                    self.stop_owned_slam()
                    self.startup_reset_phase = 'START_FRESH_SLAM'
                    self.startup_reset_retry_after = now_s + 1.0
                return False

            current_x_cm, current_y_cm, current_yaw_deg = pose
            self.target_index = 1
            self.state = 'START_STRAIGHT'
            self.start_straight_origin = None
            self.last_steering_deg = 0.0
            self.motor_ramp_start_time = time.monotonic()
            self.startup_reset_done = True
            self.startup_reset_phase = 'DONE'

            self.get_logger().info(
                f"新一圈初始化完成：map 位姿 x={current_x_cm:.2f} cm, "
                f"y={current_y_cm:.2f} cm, yaw={current_yaw_deg:.2f} deg；先直行 10 cm"
            )
            return True

        return False

    # ============================================================
    # 底盘输出
    # publish_stop：电机 PWM=1500 停止；舵机 PWM=1500 回中。
    # ============================================================
    def publish_stop(self):
        # 将电机 PWM 和舵机 PWM 填入 Twist 消息并发布到底盘驱动。
        cmd = Twist()
        cmd.linear.x = 1500.0
        cmd.angular.z = self.servo_mid
        self.cmd_pub.publish(cmd)
        # 停车时舵机已实际回中，同步变化率限制器的“上一次最终输出”。
        self.last_angular_z = self.servo_mid

    # ============================================================
    # 位姿输入与后轴参考点转换
    # /odom_combined 回调最终缓存后轴中心地面投影的 x / y / yaw。
    # ============================================================
    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)

        # 2D 运动学控制必须以后轴中心地面投影为参考点。
        # 如果 /odom_combined 本身就是 akm_rear_axle_footprint，直接使用；
        # 否则通过 TF 取“odom child -> 后轴中心”的平面偏移并旋转到 odom。
        x_m = p.x
        y_m = p.y
        child_frame = msg.child_frame_id.strip('/')
        base_frame = self.base_frame.strip('/')

        if child_frame and child_frame != base_frame:
            if (
                self.odom_rear_axle_offset_child_frame != child_frame
                or self.odom_rear_axle_offset_xy is None
            ):
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        child_frame,
                        base_frame,
                        Time(),
                    )
                    self.odom_rear_axle_offset_child_frame = child_frame
                    self.odom_rear_axle_offset_xy = (
                        tf_msg.transform.translation.x,
                        tf_msg.transform.translation.y,
                    )
                except TransformException as exc:
                    self.get_logger().warning(
                        f"等待 TF {child_frame} -> {base_frame}，暂不能换算后轴参考点: {exc}",
                        throttle_duration_sec=self.info_time_s,
                    )
                    return

            offset_x_m, offset_y_m = self.odom_rear_axle_offset_xy
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            x_m += cos_yaw * offset_x_m - sin_yaw * offset_y_m
            y_m += sin_yaw * offset_x_m + cos_yaw * offset_y_m

        self.odom_pose = (x_m * 100.0, y_m * 100.0, yaw_deg)

    def get_odom_pose(self):
        return self.odom_pose

    # ============================================================
    # 启动阶段获取车辆全局位姿
    # 返回后轴中心地面投影：current_x_cm, current_y_cm, current_yaw_deg
    # 如果当前还没有可用 TF，则返回 None。
    # ============================================================
    def get_global_pose(self):
        try:
            # 目标坐标系：map
            # 源坐标系：akm_rear_axle_footprint
            # Time() 表示读取最新可用变换。
            tf_msg = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time()
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"等待 TF {self.map_frame} -> {self.base_frame}: {exc}",
                throttle_duration_sec=self.info_time_s,
            )
            return None

        # 拆出平移和旋转四元数。
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        # 四元数转平面 yaw。
        # 这里只关心车辆绕 Z 轴的朝向，因此无需转换 roll/pitch。
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        current_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        current_yaw_deg = math.degrees(current_yaw_rad)

        # TF 是 m，保持原程序变量命名和 cm 单位
        current_x_cm = t.x * 100.0
        current_y_cm = t.y * 100.0

        return current_x_cm, current_y_cm, current_yaw_deg

    # ============================================================
    # 控制逻辑区
    # 以下按实际控制顺序连续放置：
    # 1. 当前路径段状态 -> 2. PP 前视点 -> 3. Ackermann 转角 -> 4. 主控制循环
    # ============================================================

    # ------------------------------------------------------------
    # 1) 当前目标段状态与切点判断
    # 仅用于“是否切换到下一个离散点”的判断，不限制前视点。
    # ------------------------------------------------------------
    def get_segment_state(self, current_x_cm, current_y_cm):
        """
        计算车辆相对“当前离散路径段”的位置状态。

        参数：
            current_x_cm: 后轴参考点当前 x，局部坐标，单位 cm。
            current_y_cm: 后轴参考点当前 y，局部坐标，单位 cm。

        返回：
            segment_progress_cm: 沿当前路径段方向的投影距离；超过段长表示已越过目标门线。
            lateral_distance_cm: 相对当前路径段的有符号横向偏差，单位 cm。
            passed_goal_gate: 是否已越过目标门线且横向偏差仍在允许范围内。
        """
        goal_x_cm, goal_y_cm = self.local_points_cm[self.target_index]
        start_x_cm, start_y_cm = self.local_points_cm[self.target_index - 1]

        segment_dx = goal_x_cm - start_x_cm
        segment_dy = goal_y_cm - start_y_cm
        segment_length_cm = math.hypot(segment_dx, segment_dy)

        if segment_length_cm <= 1e-6:
            return 0.0, 0.0, True

        unit_x = segment_dx / segment_length_cm
        unit_y = segment_dy / segment_length_cm

        car_from_start_x = current_x_cm - start_x_cm
        car_from_start_y = current_y_cm - start_y_cm

        segment_progress_cm = (
            car_from_start_x * unit_x
            + car_from_start_y * unit_y
        )
        lateral_distance_cm = (
            car_from_start_x * (-unit_y)
            + car_from_start_y * unit_x
        )

        passed_goal_gate = (
            segment_progress_cm >= segment_length_cm
            and abs(lateral_distance_cm) <= self.point_switch_lateral_distance
        )

        return segment_progress_cm, lateral_distance_cm, passed_goal_gate

    # ------------------------------------------------------------
    # 2) Pure Pursuit 前视点
    # 以后轴中心为圆心、lookahead_cm 为半径，搜索后续路径折线与前视圆的交点。
    # 前视点可自然跨段，不受当前线段终点限制。
    # ------------------------------------------------------------
    def get_lookahead_point(self, current_x_cm, current_y_cm, segment_progress_cm):
        """
        计算 Pure Pursuit 使用的前视点。

        参数：
            current_x_cm/current_y_cm: 后轴参考点当前局部坐标，单位 cm。
            segment_progress_cm: 车辆在当前路径段上的纵向投影进度，单位 cm；
                                 用于避免在车辆身后的路径部分寻找前视点。

        核心参数：
            self.lookahead_cm: 前视圆半径。函数会优先找“前视圆与后续折线路径”的最近前向交点。
        """
        # 至少使用 1 cm，避免 lookahead 被错误配置为 0 或负数时出现退化/除零问题。
        radius = max(self.lookahead_cm, 1.0)
        first_segment_index = max(self.target_index - 1, 0)

        best_point = None
        best_path_ahead_cm = None
        path_before_cm = 0.0

        for seg_idx in range(first_segment_index, len(self.local_points_cm) - 1):
            x1, y1 = self.local_points_cm[seg_idx]
            x2, y2 = self.local_points_cm[seg_idx + 1]
            dx = x2 - x1
            dy = y2 - y1
            seg_len = math.hypot(dx, dy)
            if seg_len <= 1e-6:
                continue

            # 只在当前段的车辆投影位置之后找交点；后续段从段首开始找。
            if seg_idx == first_segment_index:
                min_t = min(max(segment_progress_cm / seg_len, 0.0), 1.0)
            else:
                min_t = 0.0

            fx = x1 - current_x_cm
            fy = y1 - current_y_cm
            a = dx * dx + dy * dy
            b = 2.0 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - radius * radius
            discriminant = b * b - 4.0 * a * c

            if discriminant >= 0.0:
                sqrt_disc = math.sqrt(max(discriminant, 0.0))
                roots = (
                    (-b - sqrt_disc) / (2.0 * a),
                    (-b + sqrt_disc) / (2.0 * a),
                )

                for t in roots:
                    if t < min_t - 1e-9 or t > 1.0 + 1e-9:
                        continue

                    t = min(max(t, 0.0), 1.0)
                    if seg_idx == first_segment_index:
                        ahead_cm = max(0.0, (t - min_t) * seg_len)
                    else:
                        ahead_cm = path_before_cm + t * seg_len

                    if best_path_ahead_cm is None or ahead_cm < best_path_ahead_cm:
                        best_path_ahead_cm = ahead_cm
                        best_point = (x1 + t * dx, y1 + t * dy)

            if seg_idx == first_segment_index:
                path_before_cm += max(0.0, (1.0 - min_t) * seg_len)
            else:
                path_before_cm += seg_len

            # 已找到当前搜索顺序中最近的前向交点，不必再看更远的路径段。
            if best_point is not None:
                return best_point

        # 没有找到前视圆交点时（例如车辆离路径较远），不要直接跳追整条路径终点。
        # 从当前段投影位置沿折线向前走 lookahead_cm，作为稳定退化方案。
        remaining_cm = radius
        for seg_idx in range(first_segment_index, len(self.local_points_cm) - 1):
            x1, y1 = self.local_points_cm[seg_idx]
            x2, y2 = self.local_points_cm[seg_idx + 1]
            dx = x2 - x1
            dy = y2 - y1
            seg_len = math.hypot(dx, dy)
            if seg_len <= 1e-6:
                continue

            if seg_idx == first_segment_index:
                start_t = min(max(segment_progress_cm / seg_len, 0.0), 1.0)
            else:
                start_t = 0.0

            available_cm = (1.0 - start_t) * seg_len
            if remaining_cm <= available_cm:
                t = start_t + remaining_cm / seg_len
                return x1 + t * dx, y1 + t * dy

            remaining_cm -= available_cm

        return self.local_points_cm[-1]

    # ------------------------------------------------------------
    # 3) Pure Pursuit / Ackermann 转角计算
    # alpha：车头方向与前视点方向的夹角，左正右负。
    # curvature = 2*sin(alpha) / Ld
    # steering  = atan(wheelbase * curvature)
    # 返回等效前轮转角（deg），左转为正、右转为负。
    # ------------------------------------------------------------
    def pure_pursuit_steering_deg(
        self,
        current_x_cm,
        current_y_cm,
        current_yaw_deg,
        lookahead_x_cm,
        lookahead_y_cm,
    ):
        """
        根据当前位置、航向和前视点计算等效前轮转角。

        参数：
            current_x_cm/current_y_cm: 后轴中心当前局部坐标 [cm]。
            current_yaw_deg: 当前车头航向角 [deg]。
            lookahead_x_cm/lookahead_y_cm: 前视点局部坐标 [cm]。

        返回：
            steering_deg: 等效前轮转角 [deg]，左正右负。
            alpha_deg: 车头方向到前视点方向的夹角 [deg]。
            lookahead_distance_cm: 当前车辆到实际前视点的距离 [cm]。
        """
        dx_cm = lookahead_x_cm - current_x_cm
        dy_cm = lookahead_y_cm - current_y_cm
        lookahead_distance_cm = math.hypot(dx_cm, dy_cm)

        if lookahead_distance_cm <= 1e-6:
            return 0.0, 0.0, 0.0

        target_yaw_rad = math.atan2(dy_cm, dx_cm)
        current_yaw_rad = math.radians(current_yaw_deg)
        alpha_rad = target_yaw_rad - current_yaw_rad
        alpha_rad = math.atan2(math.sin(alpha_rad), math.cos(alpha_rad))

        # Pure Pursuit 曲率公式：kappa = 2*sin(alpha)/Ld，单位约 1/cm。
        curvature_per_cm = 2.0 * math.sin(alpha_rad) / lookahead_distance_cm

        # Ackermann/自行车模型：delta = atan(L * kappa)。
        # wheelbase 越长，在相同曲率要求下需要的前轮转角越大。
        steering_rad = math.atan(self.wheelbase_cm * curvature_per_cm)
        steering_deg = math.degrees(steering_rad)

        steering_deg = max(
            -self.max_steering_deg,
            min(self.max_steering_deg, steering_deg),
        )

        return (
            steering_deg,
            math.degrees(alpha_rad),
            lookahead_distance_cm,
        )

    # ------------------------------------------------------------
    # 4) 主控制循环（50 Hz）
    # 流程：
    # 1. 获取后轴中心参考点位姿；
    # 2. 判断当前离散目标点是否通过；
    # 3. 在当前段及所有后续段上寻找正常 PP 前视圆交点；
    # 4. 用 Pure Pursuit + Ackermann 自行车模型计算等效前轮转角；
    # 5. 对转角做轻量滤波并映射为舵机 PWM；
    # 6. 持续发布电机 PWM。
    # ============================================================
    def control_loop(self):
        # 节点启动后先执行一次 EKF + SLAM 复位状态机。
        # 复位期间只停车，不执行轨迹控制。
        if not self.startup_reset_step():
            return

        # 复位完成后，正常导航控制改用 /odom_combined 位姿。
        # 位姿在 odom_callback 中已统一换算到后轴参考点；这里直接使用缓存结果。
        pose = self.get_odom_pose()
        if pose is None:
            self.get_logger().warning(
                "等待 /odom_combined 位姿...",
                throttle_duration_sec=self.info_time_s,
            )
            self.publish_stop()
            return

        # 当前车辆状态：位置单位 cm，航向角单位 deg。
        current_x_cm, current_y_cm, current_yaw_deg = pose

        # 起步 1 秒内，电机 PWM 从 1600 线性缓升到 line_v。
        ramp_elapsed_s = time.monotonic() - self.motor_ramp_start_time
        ramp_ratio = min(max(ramp_elapsed_s / self.motor_ramp_duration_s, 0.0), 1.0)
        motor_pwm = self.start_motor_pwm + (self.line_v - self.start_motor_pwm) * ramp_ratio

        # ============================================================
        # 起步阶段：先保持舵机中位直行 10 cm，再进入 PP 轨迹控制。
        # ============================================================
        if self.state == "START_STRAIGHT":
            if self.start_straight_origin is None:
                self.start_straight_origin = (current_x_cm, current_y_cm)

            start_x_cm, start_y_cm = self.start_straight_origin
            straight_distance_cm = math.hypot(
                current_x_cm - start_x_cm,
                current_y_cm - start_y_cm,
            )

            if straight_distance_cm < self.start_straight_cm:
                cmd = Twist()
                cmd.linear.x = motor_pwm
                cmd.angular.z = self.servo_mid
                self.cmd_pub.publish(cmd)
                self.get_logger().info(
                    f"起步直行: {straight_distance_cm:.2f}/{self.start_straight_cm:.2f} cm, "
                    f"linear_x={cmd.linear.x:.2f}, angular_z={cmd.angular.z:.2f}",
                    throttle_duration_sec=self.info_time_s,
                )
                return

            self.state = "NAVIGATING"
            self.last_steering_deg = 0.0
            self.steering_window.clear()
            self.get_logger().info("起步直行 10 cm 完成，进入原轨迹导航")

        # ============================================================
        # 任务结束：持续发布停车
        # ============================================================
        if self.state == "FINISHED":
            self.publish_stop()
            return

        # ============================================================
        # 如果目标点已经全部完成，停车
        # ============================================================
        if self.target_index >= len(self.local_points_cm):
            self.state = "FINISHED"
            self.publish_stop()
            self.get_logger().info(
                "所有点位已完成，停车",
                throttle_duration_sec=self.info_time_s,
            )
            return

        # 当前正在追踪的目标点。
        goal_x_cm, goal_y_cm = self.local_points_cm[self.target_index]

        # 后轴参考点到当前离散目标点的二维误差，只用于到点/切点判断。
        goal_dx = goal_x_cm - current_x_cm
        goal_dy = goal_y_cm - current_y_cm

        # 与当前目标点之间的欧氏距离。
        distance_cm = math.sqrt(goal_dx * goal_dx + goal_dy * goal_dy)

        # 当前离散目标段状态只用于切点判断。
        (
            segment_progress_cm,
            lateral_distance_cm,
            passed_goal_gate,
        ) = self.get_segment_state(current_x_cm, current_y_cm)

        # 只有最后一个点需要停车。
        need_stop_point = (self.target_index == len(self.local_points_cm) - 1)

        if need_stop_point:
            reach_tolerance_cm = self.stop_tolerance_cm
        else:
            reach_tolerance_cm = self.pass_tolerance_cm

        # ============================================================
        # 到达当前目标点
        # ============================================================
        # 普通点：距离进入容差，或越过目标点门线且横向偏差允许，都切下一点。
        # 最后一个点仍保留原来的严格停车逻辑，只按 stop_tolerance_cm 停车。
        if (
            distance_cm <= reach_tolerance_cm
            or (not need_stop_point and passed_goal_gate)
        ):
            map_point = self.map_points_cm[self.target_index]
            local_point = self.local_points_cm[self.target_index]

            if self.target_index == len(self.local_points_cm) - 1:
                self.last_steering_deg = 0.0
                self.steering_window.clear()
                self.state = "FINISHED"
                self.publish_stop()

                self.get_logger().info(
                    f"已回到终点/起点 index={self.target_index}, "
                    f"地图坐标={map_point}, 局部坐标={local_point}，"
                    f"distance={distance_cm:.2f} cm，任务完成，停车",
                    throttle_duration_sec=self.info_time_s,
                )
                return

            # 普通中间点不停车，直接切换到下一个目标点。
            self.target_index += 1
            return

        # ============================================================
        # 正常导航控制：跨路径段前视点 + Pure Pursuit Ackermann 控制
        # ============================================================
        lookahead_x_cm, lookahead_y_cm = self.get_lookahead_point(
            current_x_cm,
            current_y_cm,
            segment_progress_cm,
        )

        raw_steering_deg, alpha_deg, actual_lookahead_cm = (
            self.pure_pursuit_steering_deg(
                current_x_cm,
                current_y_cm,
                current_yaw_deg,
                lookahead_x_cm,
                lookahead_y_cm,
            )
        )

        # 中值滤波 + 一阶低通仍保留，但现在滤的是“PP 前轮转角”，不是航向误差。
        # 第 1 层：3 点中值滤波，主要去掉偶发尖峰；窗口越大越平滑，但响应也越慢。
        self.steering_window.append(raw_steering_deg)
        median_steering_deg = raw_steering_deg
        if len(self.steering_window) >= 3:
            median_steering_deg = sorted(self.steering_window)[1]

        if abs(median_steering_deg) <= self.dead_zone_deg:
            steering_deg = 0.0
            self.last_steering_deg = 0.0
        else:
            # 第 2 层：一阶低通。当前值权重 0.7、上次值权重 0.3。
            # 当前值权重调大 -> 响应更快但更容易抖；上次值权重调大 -> 更平滑但转弯滞后更明显。
            steering_deg = (
                median_steering_deg * 0.7
                + self.last_steering_deg * 0.3
            )
            self.last_steering_deg = steering_deg

        steering_deg = max(
            -self.max_steering_deg,
            min(self.max_steering_deg, steering_deg),
        )

        # PP 定义：左转 steering > 0；底盘定义：左转 PWM < 1500。
        # 将“角度”转换为“舵机 PWM”。
        # 例如 steering=+5°、line_kp=23，则 PWM = 1500 - 5*23 = 1385（左转）。
        angular_z = self.servo_mid - steering_deg * self.line_kp

        if angular_z > self.servo_max:
            angular_z = self.servo_max
        elif angular_z < self.servo_min:
            angular_z = self.servo_min

        # 舵机 PWM 变化率限制：参考“幅值限幅 -> 变化率限制”的处理顺序。
        # 即使本次 angular_z 没有超过 1100~1900，也不能相对上一周期跳变过快。
        angular_z_change = angular_z - self.last_angular_z
        if angular_z_change > self.max_angular_z_change:
            angular_z = self.last_angular_z + self.max_angular_z_change
        elif angular_z_change < -self.max_angular_z_change:
            angular_z = self.last_angular_z - self.max_angular_z_change

        # 保存本次最终舵机 PWM，供下一周期继续做变化率限制。
        self.last_angular_z = angular_z

        # 保持原来的变量名 linear_x。
        linear_x = motor_pwm

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"target_index={self.target_index}, "
            f"map_goal={self.map_points_cm[self.target_index]}, "
            f"local_goal=({goal_x_cm:.2f}, {goal_y_cm:.2f}), "
            f"x={current_x_cm:.2f} cm, "
            f"y={current_y_cm:.2f} cm, "
            f"distance={distance_cm:.2f} cm, "
            f"segment_progress={segment_progress_cm:.2f} cm, "
            f"lateral_distance={lateral_distance_cm:.2f} cm, "
            f"passed_goal_gate={passed_goal_gate}, "
            f"reach_tolerance={reach_tolerance_cm:.2f} cm, "
            f"lookahead=({lookahead_x_cm:.2f}, {lookahead_y_cm:.2f}), "
            f"lookahead_cfg={self.lookahead_cm:.2f} cm, "
            f"lookahead_actual={actual_lookahead_cm:.2f} cm, "
            f"alpha={alpha_deg:.2f} deg, "
            f"raw_steering={raw_steering_deg:.2f} deg, "
            f"steering={steering_deg:.2f} deg, "
            f"max_steering={self.max_steering_deg:.2f} deg, "
            f"current_yaw={current_yaw_deg:.2f} deg, "
            f"need_stop_point={need_stop_point}, "
            f"linear_x={cmd.linear.x:.2f}, "
            f"angular_z={cmd.angular.z:.2f}",
            throttle_duration_sec=self.info_time_s,
        )



# ============================================================
# ROS2 节点入口与安全退出
# 注册 SIGINT/SIGTERM，程序终止时主动停车、舵机回中并关闭本轮 SLAM。
# ============================================================
def main(args=None):
    """
    ROS2 节点入口。

    参数：
        args: 传给 rclpy.init() 的 ROS2 命令行参数；通常直接保持 None 即可。
    """
    rclpy.init(args=args)
    node = Nav2DControl()

    # Launch 终止、Ctrl+C 等情况下的紧急停车处理。
    def emergency_stop(sig, frame):
        """
        SIGINT/SIGTERM 信号处理函数。

        参数：
            sig: 收到的系统信号编号，例如 SIGINT 或 SIGTERM。
            frame: Python 信号机制传入的当前栈帧；本函数不使用，仅为匹配回调签名。
        """
        node.get_logger().warning(
            ">>> 拦截到 Launch 终结信号，正在强制停车... <<<",
            throttle_duration_sec=node.info_time_s,
        )

        twist = Twist()
        twist.linear.x = 1500.0
        twist.angular.z = 1500.0

        # 连续发送多次停车指令，降低单帧消息丢失导致车辆继续运动的风险。
        for _ in range(5):
            node.cmd_pub.publish(twist)
            time.sleep(0.05)

        node.get_logger().info(
            "停车指令发送完毕，正在关闭本轮 SLAM",
            throttle_duration_sec=node.info_time_s,
        )
        node.stop_owned_slam()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, emergency_stop)
    signal.signal(signal.SIGTERM, emergency_stop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 异常退出也先发送停车命令，再关闭 SLAM 和 ROS 节点。
        for _ in range(5):
            try:
                node.publish_stop()
            except Exception:
                break
            time.sleep(0.05)
        node.stop_owned_slam()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

