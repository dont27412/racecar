#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``ros2 run`` 入口脚本（安装到 lib/slam_toolbox_bringup/slam_auto_switch）。

保持与包内 slam_toolbox_bringup.slam_auto_switch 的逻辑一致，只是提供
一个可直接被 ``ros2 run`` / ``ros2 launch`` 调用的入口函数。
"""

from slam_toolbox_bringup.slam_auto_switch import main  # noqa: F401

if __name__ == '__main__':
    main()
