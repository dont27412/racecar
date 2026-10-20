from glob import glob
import os

from setuptools import find_packages, setup

package_name = "ekf_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ekf_bringup maintainer",
    maintainer_email="maintainer@example.com",
    description=(
        "ROS 2 IMU/encoder preprocessing, gyro zero-bias calibration, "
        "Madgwick preprocessing and 2D robot_localization EKF bringup."
    ),
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "ekf_bringup = ekf_bringup.ekf_bringup:main",
            "encoder_raw = ekf_bringup.encoder_raw:main",
            "imu_raw_raw = ekf_bringup.imu_raw_raw:main",
            "racecar_driver = ekf_bringup.racecar_driver:main",
        ],
    },
)
