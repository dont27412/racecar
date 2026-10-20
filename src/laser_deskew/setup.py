"""laser_deskew ROS 2 Python 功能包的安装配置。

功能：
    告诉 colcon/setuptools 这个包叫什么、需要安装哪些配置/Launch 文件，
    以及 `ros2 run` 应该从哪个 Python 函数启动节点。

什么时候需要改：
    通常调雷达参数时不需要改本文件。只有修改包名、版本号、维护者信息、
    新增需要安装的配置文件，或修改可执行程序名称时才修改这里。
"""

from setuptools import find_packages, setup

# package_name
#   功能：ROS 2 Python 功能包名称。
#   怎么设置：必须和 package.xml 中 <name>、resource 文件名保持一致；正常不要单独修改。
package_name = "laser_deskew"

setup(
    # name
    #   功能：发布/安装时的 Python 包名称。
    #   怎么设置：保持与 package_name 一致。
    name=package_name,

    # version
    #   功能：功能包版本号。
    #   怎么设置：代码正式发布新版本时按项目版本规则递增；仅本地调参不需要改。
    version="0.1.0",

    # packages
    #   功能：自动查找需要安装的 Python 模块。
    #   怎么设置：一般保持 find_packages；test 不需要作为运行包安装，所以排除。
    packages=find_packages(exclude=["test"]),

    # data_files
    #   功能：把 ROS 索引、package.xml、YAML 和 Launch 文件安装到 install/.../share/laser_deskew。
    #   怎么设置：如果以后新增了必须随包安装的 config/launch 文件，需要在这里追加对应路径。
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/deskew.yaml"]),
        ("share/" + package_name + "/launch", ["launch/deskew.launch.py"]),
    ],

    # install_requires
    #   功能：声明 Python 安装阶段依赖。
    #   怎么设置：当前只需要 setuptools；ROS 消息/运行依赖放在 package.xml 中声明。
    install_requires=["setuptools"],

    # zip_safe
    #   功能：告诉 setuptools 包是否适合以压缩形式运行。
    #   怎么设置：当前保持 True 即可；通常无需为 ROS 2 节点调这个参数。
    zip_safe=True,

    # maintainer / maintainer_email
    #   功能：包维护者元数据。
    #   怎么设置：正式交付/发布时替换成真实负责人和邮箱；不影响运行算法。
    maintainer="racecar",
    maintainer_email="racecar@example.com",

    description="2D rotating LiDAR motion deskew using fused odometry.",

    # license
    #   功能：声明软件许可证。
    #   怎么设置：必须与项目实际授权方式和 package.xml 保持一致。
    license="Apache-2.0",

    # entry_points
    #   功能：注册 ROS 2 可执行入口。
    #   当前含义：执行 `ros2 run laser_deskew laser_deskew` 时，调用
    #             laser_deskew/deskew_node.py 中的 main()。
    #   怎么设置：如果修改可执行名称或入口函数，需要同步修改这里和 Launch 文件的 executable。
    entry_points={
        "console_scripts": [
            "laser_deskew = laser_deskew.deskew_node:main",
        ],
    },
)
