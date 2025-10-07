#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project Synapse — Sensors Bringup (Step 2 deliverable)
#
# This launch file brings up the primary robot sensors within ROS 2:
# - 2D LiDAR (RPLIDAR or Hokuyo/URG) for SLAM and obstacle avoidance
# - Depth/Color Camera (Intel RealSense) for vision and 3D perception
#
# It is designed to be flexible via launch arguments. You can select hardware,
# serial ports, frame IDs, and toggle optional components.
#
# Examples:
#   # RPLIDAR (USB) + RealSense:
#   ros2 launch synapse_bringup sensors.launch.py \
#       lidar_type:=rplidar lidar_port:=/dev/ttyUSB0 lidar_baud:=115200 \
#       lidar_frame_id:=laser \
#       camera_type:=realsense camera_align_depth:=true camera_pointcloud:=true
#
#   # Hokuyo URG (USB) + disable camera:
#   ros2 launch synapse_bringup sensors.launch.py \
#       lidar_type:=hokuyo lidar_port:=/dev/ttyACM0 lidar_baud:=115200 \
#       lidar_frame_id:=laser camera_type:=none
#
#   # Start only RealSense (no LiDAR):
#   ros2 launch synapse_bringup sensors.launch.py \
#       lidar_type:=none camera_type:=realsense
#
# Notes:
# - For RealSense, this includes the vendor launch (rs_launch.py) and passes
#   basic toggles like align_depth and pointcloud.enable. You can add more
#   arguments as needed (see realsense2_camera package).
# - For RPLIDAR, it includes the vendor launch (rplidar.launch.py) so you gain
#   default behavior and parameters supported by the driver.
# - For Hokuyo/URG, it launches the node directly with simple parameters.
# - Static transforms are not automatically published here; use separate launch
#   files as needed to define camera and lidar poses relative to base_link.
#
# License: Apache-2.0

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def _rplidar_group():
    """
    Include the official rplidar_ros launch file with configured arguments.
    """
    rplidar_share = get_package_share_directory("rplidar_ros")
    rplidar_launch = os.path.join(rplidar_share, "launch", "rplidar.launch.py")

    return GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rplidar_launch),
                launch_arguments={
                    "serial_port": LaunchConfiguration("lidar_port"),
                    "serial_baudrate": LaunchConfiguration("lidar_baud"),
                    "frame_id": LaunchConfiguration("lidar_frame_id"),
                    # You can add additional arguments as needed, e.g.:
                    # 'inverted': 'false',
                    # 'angle_compensate': 'true',
                }.items(),
            )
        ],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("lidar_type"), '" == "rplidar"'])
        ),
    )


def _hokuyo_group():
    """
    Launch the urg_node driver for Hokuyo/URG devices.

    Parameter names are chosen to match common urg_node conventions. Adjust
    if your distribution/package differs.
    """
    return GroupAction(
        actions=[
            Node(
                package="urg_node",
                executable="urg_node_driver",
                name="urg_node",
                output="screen",
                parameters=[
                    {
                        # Select serial (USB) by setting serial_port; leave ip_address empty.
                        "ip_address": TextSubstitution(text=""),
                        "ip_port": 10940,
                        "serial_port": LaunchConfiguration("lidar_port"),
                        "serial_baud": LaunchConfiguration("lidar_baud"),
                        # Frame name for the published LaserScan
                        "frame_id": LaunchConfiguration("lidar_frame_id"),
                        # Optional: publish intensity, etc.
                        # 'publish_intensity': True,
                    }
                ],
            )
        ],
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration("lidar_type"), '" == "hokuyo"'])
        ),
    )


def _realsense_group():
    """
    Include the official realsense2_camera launch file (rs_launch.py).

    Pass basic toggles: align_depth and pointcloud.enable. You may add or
    remap more arguments based on your camera model and needs.
    """
    rs_share = get_package_share_directory("realsense2_camera")
    rs_launch = os.path.join(rs_share, "launch", "rs_launch.py")

    return GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments={
                    # Align depth to color stream
                    "align_depth": LaunchConfiguration("camera_align_depth"),
                    # Enable pointcloud publishing
                    "pointcloud.enable": LaunchConfiguration("camera_pointcloud"),
                    # Suggested defaults; uncomment/customize as needed:
                    # 'rgb_camera.profile': '640x480x30',
                    # 'depth_module.profile': '640x480x30',
                    # 'enable_infra1': 'false',
                    # 'enable_infra2': 'false',
                    # 'enable_sync': 'true',
                }.items(),
            )
        ],
        condition=IfCondition(
            PythonExpression(
                ['"', LaunchConfiguration("camera_type"), '" == "realsense"']
            )
        ),
    )


def generate_launch_description() -> LaunchDescription:
    # General frames
    base_frame = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Robot base frame (parent for sensor frames)",
    )

    # LiDAR selection and parameters
    lidar_type = DeclareLaunchArgument(
        "lidar_type",
        default_value="rplidar",
        choices=["rplidar", "hokuyo", "none"],
        description="LiDAR driver to start (rplidar, hokuyo, or none)",
    )
    lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value="/dev/ttyUSB0",
        description="Serial device for LiDAR (if applicable)",
    )
    lidar_baud = DeclareLaunchArgument(
        "lidar_baud",
        default_value="115200",
        description="Serial baudrate for LiDAR (if applicable)",
    )
    lidar_frame_id = DeclareLaunchArgument(
        "lidar_frame_id",
        default_value="laser",
        description="Frame ID for the published LaserScan",
    )

    # Camera selection and toggles
    camera_type = DeclareLaunchArgument(
        "camera_type",
        default_value="realsense",
        choices=["realsense", "none"],
        description="Camera driver to start (realsense or none)",
    )
    camera_align_depth = DeclareLaunchArgument(
        "camera_align_depth",
        default_value="true",
        description="If true, align depth to color image (RealSense only)",
    )
    camera_pointcloud = DeclareLaunchArgument(
        "camera_pointcloud",
        default_value="true",
        description="If true, enable RealSense pointcloud publishing",
    )

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(base_frame)

    ld.add_action(lidar_type)
    ld.add_action(lidar_port)
    ld.add_action(lidar_baud)
    ld.add_action(lidar_frame_id)

    ld.add_action(camera_type)
    ld.add_action(camera_align_depth)
    ld.add_action(camera_pointcloud)

    # Groups to start chosen sensors
    ld.add_action(_rplidar_group())
    ld.add_action(_hokuyo_group())
    ld.add_action(_realsense_group())

    return ld
