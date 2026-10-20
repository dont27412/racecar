# slam_toolbox_bringup

用于启动 ROS 2 `slam_toolbox` 在线异步建图的简单 bringup 功能包。

本版本已为主要工程文件补充中文注释，未修改原有功能逻辑：

- `package.xml`：功能包元数据与依赖说明。
- `CMakeLists.txt`：构建及安装规则说明。
- `launch/slam_toolbox_bringup.launch.py`：启动流程、参数传递及路径获取说明。
- `config/slam_toolbox_bringup.yaml`：SLAM、扫描匹配、回环检测、地图/TF 等参数说明。

## 启动示例


