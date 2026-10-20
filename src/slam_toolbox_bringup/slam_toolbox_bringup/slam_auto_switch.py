#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""建图 -> 自动存图 -> 自动切换为定位（localization）模式。

工作流程
========
1. 启动 ``map_and_localization_slam_toolbox_node``（本包的 launch 负责），
   初始为 **mapping** 模式，开始建图。
2. 惯性导航小车经过某个预设点时会发布一条::

       /slam_switch_trigger  (std_msgs/String)

   消息内容可以是任意字符串（例如 ``"save"`` 或 ``"done"``）。
3. 本节点收到该消息后，自动依次执行：

   a. 调用 slam_toolbox 的 ``/slam_toolbox/save_map`` 服务，把当前地图
      保存到默认位置（``~/maps/<map_name>``，同时生成 ``.pgm`` + ``.yaml``）。
   b. 通过 ``/slam_toolbox/set_parameters`` 把 slam_toolbox 的 ``mode``
      参数从 ``mapping`` 改为 ``localization``——即从“建图”直接切换为
      “定位”，**无需关闭 slam_toolbox、也无需启动 AMCL**。
   c. 可选地调用 ``serialize_map`` 保存位姿图（便于以后继续优化）。

整个过程不需要重启节点，切换通常在 1 秒以内完成。

实现要点（很关键）
==================
ROS 2 的服务是异步的：**不能在订阅回调里使用阻塞式
``rclpy.spin_until_future_complete``**（那时已经处于执行器的 spin 循环内，
阻塞会卡死/抛异常）。因此本节点用**回调链**的方式实现顺序流程：

    触发回调 -> save_map 请求 -> save_map 完成回调
             -> serialize 请求 -> serialize 完成回调
             -> set_parameters(mode) 请求 -> 完成回调 -> 打印结果

每一步都在“上一步完成回调”里发起下一步，天然串行且不阻塞。

为什么不用 AMCL？
=================
``map_and_localization_slam_toolbox_node`` 的 localization 模式本身就会持续
发布 ``map -> odom`` 的 TF（相当于内置定位器），Nav2 直接使用这条 TF 即可
完成标点导航，因此不需要再起 AMCL。

重要前提
========
* 运行时切换 ``mode`` 参数需要 slam_toolbox 版本支持。若版本不支持，本节点
  会在日志中报错；此时请改用 localization 版 yaml 重启 slam_toolbox。
* 切换时不要重启节点、不要移动机器人，这样位姿连续，可“就地”进入定位模式，
  无需重新给初始位姿。
"""

import os

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String

# slam_toolbox 提供的保存服务。优先从 slam_toolbox.srv 导入；若当前环境未
# source slam_toolbox，则该模块不存在，此时给出明确提示。
try:
    from slam_toolbox.srv import SaveMap  # type: ignore
    from slam_toolbox.srv import SerializePoseGraph  # type: ignore
    _HAS_SLAM_SRV = True
except Exception:  # pragma: no cover - 取决于是否 source 了 slam_toolbox
    _HAS_SLAM_SRV = False


class SlamAutoSwitch(Node):
    """订阅 /slam_switch_trigger，自动存图并切换 SLAM 到定位模式。"""

    def __init__(self):
        super().__init__('slam_auto_switch')

        # ------------------------------------------------------------------
        # 参数
        # ------------------------------------------------------------------
        self.declare_parameter('trigger_topic', '/slam_switch_trigger')
        self.declare_parameter('slam_node_name', '/slam_toolbox')
        self.declare_parameter('save_dir', os.path.expanduser('~/maps'))
        self.declare_parameter('map_name', 'my_map')
        self.declare_parameter('target_mode', 'localization')
        self.declare_parameter('serialize', True)
        self.declare_parameter('save_timeout', 10.0)
        self.declare_parameter('trigger_once', True)

        self._trigger_topic = self.get_parameter(
            'trigger_topic').get_parameter_value().string_value
        self._slam_node = self.get_parameter(
            'slam_node_name').get_parameter_value().string_value
        self._save_dir = self.get_parameter(
            'save_dir').get_parameter_value().string_value
        self._map_name = self.get_parameter(
            'map_name').get_parameter_value().string_value
        self._target_mode = self.get_parameter(
            'target_mode').get_parameter_value().string_value
        self._serialize = self.get_parameter(
            'serialize').get_parameter_value().bool_value
        self._save_timeout = self.get_parameter(
            'save_timeout').get_parameter_value().double_value
        self._trigger_once = self.get_parameter(
            'trigger_once').get_parameter_value().bool_value

        slam_base = self._slam_node.rstrip('/')
        self._save_map_srv = f'{slam_base}/save_map'
        self._serialize_srv = f'{slam_base}/serialize_map'
        self._set_params_srv = f'{slam_base}/set_parameters'

        self._map_path = os.path.join(self._save_dir, self._map_name)
        self._done = False

        # ------------------------------------------------------------------
        # 服务客户端
        # ------------------------------------------------------------------
        if _HAS_SLAM_SRV:
            self._save_client = self.create_client(SaveMap, self._save_map_srv)
            self._serialize_client = self.create_client(
                SerializePoseGraph, self._serialize_srv)
        else:
            self._save_client = None
            self._serialize_client = None
            self.get_logger().error(
                '未能导入 slam_toolbox.srv（SaveMap/SerializePoseGraph）。'
                '请确认已 source slam_toolbox 工作空间后再启动本节点。')

        self._set_params_client = self.create_client(
            SetParameters, self._set_params_srv)

        # ------------------------------------------------------------------
        # 订阅触发话题
        # ------------------------------------------------------------------
        self.create_subscription(
            String, self._trigger_topic, self._on_trigger, 10)

        self.get_logger().info(
            f'slam_auto_switch 已就绪：订阅 "{self._trigger_topic}"；'
            f'收到触发后将保存地图到 "{self._map_path}" 并切换到 '
            f'"{self._target_mode}" 模式。')

    # ----------------------------------------------------------------------
    # 触发回调（只负责发起第一步，绝不阻塞）
    # ----------------------------------------------------------------------
    def _on_trigger(self, msg: String):
        if self._done and self._trigger_once:
            self.get_logger().warn(
                f'已执行过切换，忽略重复触发："{msg.data}"')
            return
        if self._done:
            # trigger_once=False 时允许再次触发。
            self._done = False

        self.get_logger().info(
            f'收到 /slam_switch_trigger："{msg.data}"，开始保存地图 ...')
        os.makedirs(self._save_dir, exist_ok=True)
        self._start_save_map()

    # ----------------------------------------------------------------------
    # 步骤 1：save_map
    # ----------------------------------------------------------------------
    def _start_save_map(self):
        if self._save_client is None:
            self.get_logger().error('save_map 客户端不可用，终止流程。')
            return

        if not self._save_client.service_is_ready():
            self.get_logger().error(
                f'服务 "{self._save_map_srv}" 尚未就绪，终止流程。')
            return

        request = SaveMap.Request()
        # SaveMap 请求字段为 std_msgs/String name。
        request.name.data = self._map_path

        future = self._save_client.call_async(request)
        future.add_done_callback(self._on_save_map_done)

    def _on_save_map_done(self, future):
        # slam_toolbox 的 SaveMap 服务响应里没有 result 字段，
        # 因此绝不能靠响应内容判断成败——之前用 getattr(result, 'result')
        # 会误判，导致地图明明已保存却错误地报“保存地图失败”。
        # 正确判据：
        #   1) future 没有抛异常（未被取消 / 远端没有返回错误）；
        #   2) future.result() 非 None，说明服务端已完成本次调用。
        # 满足以上两点即认为地图已成功保存（.pgm / .yaml 已落盘）。
        try:
            result = future.result()
        except Exception as exc:  # 服务被取消 / 远端错误 / 超时等
            self.get_logger().error(
                f'保存地图调用异常，终止流程：{exc}')
            return

        if result is None:
            self.get_logger().error(
                '保存地图失败（服务无响应），终止流程。')
            return

        self.get_logger().info(
            f'已保存地图：{self._map_path}.pgm / {self._map_path}.yaml')

        # 进入下一步。
        if self._serialize:
            self._start_serialize()
        else:
            self._start_switch_mode()

    # ----------------------------------------------------------------------
    # 步骤 2（可选）：serialize_map
    # ----------------------------------------------------------------------
    def _start_serialize(self):
        if self._serialize_client is None or \
                not self._serialize_client.service_is_ready():
            self.get_logger().warn(
                'serialize_map 服务不可用，跳过位姿图保存。')
            self._start_switch_mode()
            return

        request = SerializePoseGraph.Request()
        request.filename = self._map_path
        future = self._serialize_client.call_async(request)
        future.add_done_callback(self._on_serialize_done)

    def _on_serialize_done(self, future):
        if future.result() is not None:
            self.get_logger().info(
                f'已保存位姿图：{self._map_path}.posegraph / .data')
        else:
            self.get_logger().warn('位姿图保存失败（不影响模式切换）。')
        self._start_switch_mode()

    # ----------------------------------------------------------------------
    # 步骤 3：切换 mode
    # ----------------------------------------------------------------------
    def _start_switch_mode(self):
        if not self._set_params_client.service_is_ready():
            self.get_logger().error(
                f'服务 "{self._set_params_srv}" 尚未就绪，无法切换模式。')
            return

        param = ParameterMsg()
        param.name = 'mode'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value=self._target_mode,
        )

        request = SetParameters.Request()
        request.parameters = [param]

        future = self._set_params_client.call_async(request)
        future.add_done_callback(self._on_switch_mode_done)

    def _on_switch_mode_done(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error('set_parameters 调用失败（无返回）。')
            return

        results = result.results
        if results and results[0].successful:
            self._done = True
            self.get_logger().info(
                '==================================================')
            self.get_logger().info(f'  地图已保存到 : {self._map_path}')
            self.get_logger().info(f'  SLAM 已切换为 : {self._target_mode}')
            self.get_logger().info(
                '  现在可以启动 Nav2 并开始标点导航（无需 AMCL）。')
            self.get_logger().info(
                '==================================================')
            return

        reason = results[0].reason if results else '未知原因'
        self.get_logger().error(
            f'切换 mode 失败：{reason}\n'
            '提示：部分 slam_toolbox 版本不支持运行时改 mode。'
            '此时请改用 localization 版 yaml 重启 slam_toolbox。')


def main(args=None):
    rclpy.init(args=args)
    node = SlamAutoSwitch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
