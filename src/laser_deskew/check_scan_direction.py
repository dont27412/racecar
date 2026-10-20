#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


# ============================================================
# 可调参数
# ============================================================

# TOPIC
#   功能：指定用于判断扫描方向的原始 PointCloud2 话题。
#   参数作用：脚本会按照“点在消息中的原始顺序”统计相邻点方位角是递增还是递减。
#   怎么设置：
#     1. 运行 `ros2 topic list` 查找雷达原始点云话题；
#     2. 运行 `ros2 topic type <话题名>`，确认类型是 sensor_msgs/msg/PointCloud2；
#     3. 如果你的驱动不是 /x10/lslidar_point_cloud，就改成实际话题。
TOPIC = '/x10/lslidar_point_cloud'

# CHECK_FRAMES
#   功能：连续统计多少帧后再输出最终结论。
#   参数作用：帧数越多，结果越不容易被单帧异常点影响，但检测时间也越长。
#   怎么设置：一般 10~30 帧即可；当前 20 帧适合快速且相对稳定地判断。
CHECK_FRAMES = 20

# MAX_VALID_STEP_DEG
#   功能：相邻点允许参与统计的最大角度变化，单位 °。
#   参数作用：过滤缺点、异常点、不同数据块切换造成的大角度跳变。
#   怎么设置：应明显大于正常相邻点角分辨率，又明显小于异常跳变；
#             2D 雷达可先保持 20°。若有效点太少，可适当增大；若异常点很多，可适当减小。
MAX_VALID_STEP_DEG = 20.0

# MIN_VALID_STEP_DEG
#   功能：相邻点允许参与统计的最小角度变化，单位 °。
#   参数作用：过滤重复点和浮点数微小抖动，避免把“几乎没动”误判为方向。
#   怎么设置：应远小于雷达正常角分辨率；0.001° 通常无需修改。
MIN_VALID_STEP_DEG = 0.001


def angle_deg(x, y):
    """把点坐标转换成 0~360° 方位角。

    功能：用于比较 PointCloud2 中相邻点的扫描角变化方向。
    参数：x、y 为点在雷达坐标系中的平面坐标。
    返回：范围 [0, 360) 的角度值，单位 °。
    """
    a = math.degrees(math.atan2(y, x))

    if a < 0:
        a += 360.0

    return a


def signed_angle_diff(a2, a1):
    """计算从 a1 到 a2 的有符号角度变化。

    功能：正确处理 359° -> 0° 这种跨越 360° 的情况。
    参数：a1 为前一个点角度，a2 为后一个点角度，单位 °。
    返回：约位于 [-180, 180)；正数表示方位角递增，负数表示递减。

    例如：359 -> 1 = +2；1 -> 359 = -2。
    """
    return (a2 - a1 + 180.0) % 360.0 - 180.0


class ScanDirectionChecker(Node):
    """判断 deskew.yaml 中 raw_azimuth_increases 应该如何设置。

    功能：统计原始 PointCloud2 中相邻点的方位角变化方向。

    结果怎么用：
        - 若最终显示“原始方位角随时间总体递增”，设置 raw_azimuth_increases: true；
        - 若显示“总体递减”，设置 raw_azimuth_increases: false；
        - 若无法可靠判断，说明点云可能被驱动重排，不能直接用该脚本下结论。
    """

    def __init__(self):
        super().__init__('scan_direction_checker')

        # 使用传感器数据 QoS，尽量与雷达驱动常用的 best-effort 配置兼容。
        self.sub = self.create_subscription(
            PointCloud2,
            TOPIC,
            self.callback,
            qos_profile_sensor_data
        )

        # 跨帧累计统计量，用于降低单帧异常点对结论的影响。
        self.frame_count = 0

        self.total_positive = 0
        self.total_negative = 0

        self.total_positive_angle = 0.0
        self.total_negative_angle = 0.0

        print()
        print('==========================================')
        print(' 雷达原始方位角方向检测')
        print('==========================================')
        print(f'Topic       : {TOPIC}')
        print(f'统计帧数    : {CHECK_FRAMES}')
        print()
        print('正在等待雷达数据...')
        print()


    def callback(self, msg):
        """处理一帧点云并统计相邻点的方位角递增/递减次数。"""

        # 只读取判断扫描方向所需的 x、y 字段，并跳过 NaN 点。
        pts = point_cloud2.read_points_list(
            msg,
            field_names=('x', 'y'),
            skip_nans=True
        )

        if len(pts) < 2:
            print('这一帧有效点太少，跳过')
            return

        # 按消息中的原始点顺序计算每个点的 0~360° 方位角。
        # 这里不能排序，否则会破坏“随采样时间变化”的顺序信息。
        angles = []

        for p in pts:
            a = angle_deg(p.x, p.y)
            angles.append(a)

        positive = 0
        negative = 0

        positive_angle = 0.0
        negative_angle = 0.0

        # 保存有效角度步长，便于调试；最终判定主要使用正/负步长计数。
        valid_steps = []

        # -----------------------------------
        # 判断消息里相邻点的角度变化方向
        # -----------------------------------

        for i in range(1, len(angles)):

            diff = signed_angle_diff(
                angles[i],
                angles[i - 1]
            )

            # 过小变化通常只是重复点/数值误差，不用于方向统计。
            if abs(diff) < MIN_VALID_STEP_DEG:
                continue

            # 过大跳变通常来自缺点、异常点或点云结构切换，也不用于方向统计。
            if abs(diff) > MAX_VALID_STEP_DEG:
                continue

            valid_steps.append(diff)

            if diff > 0:
                positive += 1
                positive_angle += diff
            else:
                negative += 1
                negative_angle += abs(diff)

        if positive + negative == 0:
            print('没有足够有效的相邻点，跳过这一帧')
            return

        self.frame_count += 1

        self.total_positive += positive
        self.total_negative += negative

        self.total_positive_angle += positive_angle
        self.total_negative_angle += negative_angle

        frame_total = positive + negative

        pos_ratio = positive / frame_total
        neg_ratio = negative / frame_total

        # -----------------------------------
        # 显示当前帧
        # -----------------------------------

        print(
            f'帧 {self.frame_count:02d}: '
            f'点数={len(pts):5d}  '
            f'递增={positive:5d} ({pos_ratio * 100:5.1f}%)  '
            f'递减={negative:5d} ({neg_ratio * 100:5.1f}%)'
        )

        # 第一帧打印前 30 个角度方便人工检查
        if self.frame_count == 1:

            print()
            print('前 30 个点的方位角：')

            for i, a in enumerate(angles[:30]):
                print(
                    f'{i:4d}    {a:8.2f}°'
                )

            print()

        # -----------------------------------
        # 达到统计帧数
        # -----------------------------------

        if self.frame_count >= CHECK_FRAMES:
            self.show_result()
            rclpy.shutdown()


    def show_result(self):
        """汇总所有有效帧，并输出 deskew.yaml 的推荐参数值。"""

        total = (
            self.total_positive
            + self.total_negative
        )

        if total == 0:
            print('无法判断扫描方向')
            return

        positive_ratio = (
            self.total_positive / total
        )

        negative_ratio = (
            self.total_negative / total
        )

        print()
        print()
        print('==========================================')
        print(' 最终检测结果')
        print('==========================================')
        print()

        print(
            f'有效相邻点总数 : {total}'
        )

        print(
            f'方位角递增     : '
            f'{self.total_positive} '
            f'({positive_ratio * 100:.2f}%)'
        )

        print(
            f'方位角递减     : '
            f'{self.total_negative} '
            f'({negative_ratio * 100:.2f}%)'
        )

        print()

        # -----------------------------------
        # 做最终判断
        # 80% 阈值用于给噪声/少量异常点留出余量。
        # -----------------------------------

        if positive_ratio >= 0.80:

            print('结论：')
            print()
            print('    原始方位角随时间总体递增')
            print('    0° -> 90° -> 180° -> 270° -> 360°')
            print()
            print('deskew.yaml 建议设置：')
            print()
            print('    raw_azimuth_increases: true')

        elif negative_ratio >= 0.80:

            print('结论：')
            print()
            print('    原始方位角随时间总体递减')
            print('    360° -> 270° -> 180° -> 90° -> 0°')
            print()
            print('deskew.yaml 建议设置：')
            print()
            print('    raw_azimuth_increases: false')

        else:

            print('结论：')
            print()
            print('    无法可靠判断扫描方向。')
            print()
            print('可能原因：')
            print('    1. PointCloud2 中的点被驱动重新排序')
            print('    2. 点云存在多个扫描层/特殊排列')
            print('    3. 数据异常或跳点很多')
            print()
            print('这种情况下不能单靠 PointCloud2 点顺序')
            print('判断 raw_azimuth_increases。')

        print()
        print('==========================================')


def main():

    rclpy.init()

    node = ScanDirectionChecker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            rclpy.shutdown()

        node.destroy_node()


if __name__ == '__main__':
    main()