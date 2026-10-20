cmake_minimum_required(VERSION 3.5)
project(lslidar_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# X10-only: lslidar_x10_driver only needs the raw packet message.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LslidarPacket.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()
