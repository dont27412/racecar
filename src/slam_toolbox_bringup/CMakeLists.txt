# ============================================================
# slam_toolbox_bringup 构建/安装配置
# ============================================================
# 功能：把本包的 launch/ 和 config/ 文件安装到 ROS 2 的 install 空间，
#       让 `ros2 launch` 和 get_package_share_directory() 能找到它们。
#
# 怎么设置：正常调 SLAM 参数时不需要改本文件；主要修改 config/*.yaml。

# 功能：声明构建本包所需的最低 CMake 版本。
# 怎么设置：只要当前 ROS 2 / 系统 CMake 满足即可；没有兼容性需求不要随意提高。
cmake_minimum_required(VERSION 3.8)

# 功能：定义功能包工程名称。
# 怎么设置：必须和 package.xml 中 <name> 完全一致，否则 ROS 2 包结构会不一致。
project(slam_toolbox_bringup)

# 功能：声明本包使用 ament_cmake 构建。怎么设置：当前 CMake 包保持不变。
find_package(ament_cmake REQUIRED)

# ------------------------------------------------------------
# 安装 Python 节点源码（slam_auto_switch）
# ------------------------------------------------------------
# 功能：把可执行 Python 脚本安装到 lib/<包名>/，让 `ros2 run` / `ros2 launch`
#       能直接找到它。
# 怎么设置：新增 Python 可执行脚本时，在 FILES 后追加对应 .py 文件名。
install(
  PROGRAMS
    slam_toolbox_bringup/slam_auto_switch_main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME slam_auto_switch
)

# 功能：把 Python 模块本身也安装到 lib/<包名>/，供 node 内部 import 使用。
# 怎么设置：新增被 import 的 .py 文件时追加到 FILES 后。
install(
  FILES
    slam_toolbox_bringup/__init__.py
    slam_toolbox_bringup/slam_auto_switch.py
  DESTINATION lib/${PROJECT_NAME}/slam_toolbox_bringup
)

# 功能：把 launch/ 和 config/ 两个目录安装到 share/<包名>。
# 怎么设置：如果以后新增 rviz/、maps/ 等也需要安装的目录，可把目录名追加到 DIRECTORY 后。
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# 功能：完成 ament 包导出/注册。
# 怎么设置：通常必须位于 CMakeLists.txt 末尾，不需要修改参数。
ament_package()

