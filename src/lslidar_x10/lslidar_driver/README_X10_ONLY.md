# X10 单独编译版

本包来自 src_V1.0(20260920-121754).zip。压缩包包含 lslidar_driver 和
lslidar_msgs 两个 ROS 2 包，放置位置与原工作空间中的两个包一致。

## 修改范围

1. CMakeLists.txt 不再编译 lslidar_ch_driver.cpp、lslidar_cx_driver.cpp、
   lslidar_ls_driver.cpp；继续编译 X10 和公共输入、串口、服务、设备信息代码。
2. src/lslidar_driver_node.cpp 去掉 CH/CX/LS 头文件和实例化分支，
   lidar_type 默认值改为 X10。配置其他类型时明确报错。
3. lslidar_msgs 保持原样：公共基类和服务头文件仍引用其中的消息和服务，
   本次没有拆分这些公共代码，不能直接删除 srv 接口生成项。

原有其他型号的源码、配置、launch 文件保留以便恢复，但本版本的节点只支持
lidar_type: X10；不要使用 CH/CX/LS 启动文件。

X10 内的 M10GPS 等型号解析保持原样。当前 lslidar_x10.yaml 仍配置为
M10GPS、/dev/laser、发布 /scan。没有修改时间戳、角度、线程或去畸变逻辑，
也没有修改优化等级。本包不包含此前审查提到的运行问题修复。

## 替换与编译

1. 停止正在运行的雷达节点。
2. 将工作空间 src 下原有的 lslidar_driver 和 lslidar_msgs 备份到工作空间之外，
   再将压缩包里的同名两个文件夹放回 src。不要在工作空间中保留两份同名 ROS 包。
3. 打开已加载所用 ROS 2 发行版环境的终端，进入工作空间根目录（src 的上一级）。
4. 执行：

```bash
colcon build --packages-up-to lslidar_driver --cmake-force-configure
source install/setup.bash
ros2 launch lslidar_driver lslidar_x10_launch.py
```

如果原工作空间使用 --merge-install，请沿用该选项；其他安装方式选项也应与
原工作空间保持一致。无需为本次修改清空整个 build 或 install。
--cmake-force-configure 会重新生成构建规则，使删除的编译目标源文件生效。
旧的 CH/CX/LS 对象文件即使还留在 build 内，也不再属于新的目标源列表。

驱动和消息包成功构建后，日常没有修改它们时可跳过：

```bash
colcon build --packages-skip lslidar_driver lslidar_msgs
```

## 验证与限制

交付前已核对本地头文件依赖、消息/服务接口引用及文件差异；
X10 解析实现、launch、YAML 和整个 lslidar_msgs 与上传版本逐字节一致。
当前检查环境没有 ROS 2/colcon，未执行真实 ROS 2 编译、链接或雷达测试。
请在原有依赖齐全的机器上构建，确认 /scan 持续发布且雷达工作正常。
编译速度提升主要出现在完整重编译；没有实测耗时，不承诺提速比例。
