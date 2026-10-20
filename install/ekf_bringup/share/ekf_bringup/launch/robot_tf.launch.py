#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Publish the robot's static TF tree.

Coordinate convention (REP-103):
  +X forward, +Y left, +Z up.

Measured geometry:
  vehicle size: 0.48 m long x 0.35 m wide x 0.28 m high
  vehicle geometric center (base_link): 0.14 m above ground
  LiDAR: 0.16 m behind the front edge, on the centerline, 0.26 m above ground
  IMU:   0.14 m behind the front edge, 0.14 m from the left edge, 0.18 m above ground

Ackermann kinematic reference point:
  akm_rear_axle_footprint = rear axle center projected onto the ground
  base_footprint is 0.170 m forward of akm_rear_axle_footprint

Therefore, relative to base_link:
  laser_link = (0.080, 0.000, 0.120) m
  imu_link   = (0.100, 0.035, 0.040) m

No mounting attitude offset was provided, so roll/pitch/yaw are all set to zero.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def static_tf(name, parent, child, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", str(x),
            "--y", str(y),
            "--z", str(z),
            "--roll", str(roll),
            "--pitch", str(pitch),
            "--yaw", str(yaw),
            "--frame-id", parent,
            "--child-frame-id", child,
        ],
        output="screen",
    )


def generate_launch_description():
    # Ackermann kinematic reference point (rear axle center projected onto ground)
    # -> base_footprint (ground projection of the geometric center).
    # Wheelbase is 0.34 m and the front/rear axles are symmetric about the geometric center,
    # so base_footprint is 0.17 m ahead of akm_rear_axle_footprint.
    rear_axle_to_base_footprint = static_tf(
        "rear_axle_to_base_footprint",
        "akm_rear_axle_footprint",
        "base_footprint",
        0.17,
        0.0,
        0.0,
    )

    # Ground projection of the geometric center -> geometric center (base_link).
    base_footprint_to_base_link = static_tf(
        "base_footprint_to_base_link",
        "base_footprint",
        "base_link",
        0.0,
        0.0,
        0.14,
    )

    # Geometric center (base_link) -> IMU.
    base_link_to_imu_link = static_tf(
        "base_link_to_imu_link",
        "base_link",
        "imu_link",
        0.10,
        0.035,
        0.04,
    )

    # Geometric center (base_link) -> LiDAR.
    base_link_to_laser_link = static_tf(
        "base_link_to_laser_link",
        "base_link",
        "laser_link",
        0.08,
        0.0,
        0.12,
    )

    return LaunchDescription([
        rear_axle_to_base_footprint,
        base_footprint_to_base_link,
        base_link_to_imu_link,
        base_link_to_laser_link,
    ])
