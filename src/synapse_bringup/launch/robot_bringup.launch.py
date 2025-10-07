#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project Synapse — Robot Bringup (All-in-one)
#
# This launch file composes:
# 1) Robot description (URDF via robot_state_publisher)
# 2) Sensors bringup (LiDAR + Camera)
# 3) Core stack (base driver, memory, perception, emotion, display, interaction, behaviors)
#
# Examples:
#   # Start everything with defaults
#   ros2 launch synapse_bringup robot_bringup.launch.py
#
#   # Skip sensors (e.g., when running in a sim or already launched elsewhere)
#   ros2 launch synapse_bringup robot_bringup.launch.py start_sensors:=false
#
#   # Override serial/camera params
#   ros2 launch synapse_bringup robot_bringup.launch.py \
#       base_serial_port:=/dev/ttyUSB1 \
#       lidar_type:=rplidar lidar_port:=/dev/ttyUSB0 \
#       camera_type:=realsense
#
#   # Use a namespace
#   ros2 launch synapse_bringup robot_bringup.launch.py namespace:=synapse
#
# Notes:
# - This wrapper includes sensors explicitly and disables sensors in the core stack
#   (passes start_sensors:=false to core.launch.py) to avoid double bringup.
#

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _include_description():
    desc_share = get_package_share_directory("synapse_description")
    desc_launch = os.path.join(desc_share, "launch", "description.launch.py")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(desc_launch),
        launch_arguments={
            # Pass-through of some common knobs; the xacro file defaults inside the description package.
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "publish_joint_state": LaunchConfiguration("publish_joint_state"),
            "use_joint_state_publisher_gui": LaunchConfiguration(
                "use_joint_state_publisher_gui"
            ),
        }.items(),
    )


def _include_sensors():
    bringup_share = get_package_share_directory("synapse_bringup")
    sensors_launch = os.path.join(bringup_share, "launch", "sensors.launch.py")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch),
        launch_arguments={
            # LiDAR
            "lidar_type": LaunchConfiguration("lidar_type"),
            "lidar_port": LaunchConfiguration("lidar_port"),
            "lidar_baud": LaunchConfiguration("lidar_baud"),
            "lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
            # Camera
            "camera_type": LaunchConfiguration("camera_type"),
            "camera_align_depth": LaunchConfiguration("camera_align_depth"),
            "camera_pointcloud": LaunchConfiguration("camera_pointcloud"),
        }.items(),
    )


def _include_core():
    bringup_share = get_package_share_directory("synapse_bringup")
    core_launch = os.path.join(bringup_share, "launch", "core.launch.py")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_launch),
        launch_arguments={
            # Global
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "namespace": LaunchConfiguration("namespace"),
            # Explicitly disable sensors in core; sensors are brought up by this wrapper
            "start_sensors": "false",
            # Module toggles
            "enable_base": LaunchConfiguration("enable_base"),
            "enable_memory": LaunchConfiguration("enable_memory"),
            "enable_perception": LaunchConfiguration("enable_perception"),
            "enable_emotion": LaunchConfiguration("enable_emotion"),
            "enable_display": LaunchConfiguration("enable_display"),
            "enable_interaction": LaunchConfiguration("enable_interaction"),
            "enable_behaviors": LaunchConfiguration("enable_behaviors"),
            # Perception topics
            "image_topic": LaunchConfiguration("image_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            # Base parameters
            "base_cmd_vel_topic": LaunchConfiguration("base_cmd_vel_topic"),
            "base_serial_port": LaunchConfiguration("base_serial_port"),
            "base_serial_baud": LaunchConfiguration("base_serial_baud"),
            "base_wheel_radius": LaunchConfiguration("base_wheel_radius"),
            "base_wheel_separation": LaunchConfiguration("base_wheel_separation"),
        }.items(),
    )


def generate_launch_description() -> LaunchDescription:
    # Common arguments
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time from /clock",
    )
    namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace for core nodes"
    )

    # Description toggles/args
    start_description = DeclareLaunchArgument(
        "start_description",
        default_value="true",
        description="Start robot description publisher",
    )
    robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="synapse_base",
        description="Robot name/namespace in description",
    )
    publish_joint_state = DeclareLaunchArgument(
        "publish_joint_state",
        default_value="false",
        description="Start joint_state_publisher",
    )
    use_joint_state_publisher_gui = DeclareLaunchArgument(
        "use_joint_state_publisher_gui",
        default_value="false",
        description="Use GUI variant",
    )

    # Sensors toggles/args
    start_sensors = DeclareLaunchArgument(
        "start_sensors",
        default_value="true",
        description="Start sensors bringup (LiDAR + Camera)",
    )
    lidar_type = DeclareLaunchArgument(
        "lidar_type",
        default_value="rplidar",
        description="LiDAR driver: rplidar | hokuyo | none",
    )
    lidar_port = DeclareLaunchArgument(
        "lidar_port", default_value="/dev/ttyUSB0", description="LiDAR serial device"
    )
    lidar_baud = DeclareLaunchArgument(
        "lidar_baud", default_value="115200", description="LiDAR serial baudrate"
    )
    lidar_frame_id = DeclareLaunchArgument(
        "lidar_frame_id", default_value="laser", description="Frame ID for LaserScan"
    )

    camera_type = DeclareLaunchArgument(
        "camera_type",
        default_value="realsense",
        description="Camera driver: realsense | none",
    )
    camera_align_depth = DeclareLaunchArgument(
        "camera_align_depth",
        default_value="true",
        description="Align depth to color (RealSense)",
    )
    camera_pointcloud = DeclareLaunchArgument(
        "camera_pointcloud",
        default_value="true",
        description="Enable pointcloud from camera",
    )

    # Core toggles (forwarded into core.launch.py)
    enable_base = DeclareLaunchArgument("enable_base", default_value="true")
    enable_memory = DeclareLaunchArgument("enable_memory", default_value="true")
    enable_perception = DeclareLaunchArgument("enable_perception", default_value="true")
    enable_emotion = DeclareLaunchArgument("enable_emotion", default_value="true")
    enable_display = DeclareLaunchArgument("enable_display", default_value="true")
    enable_interaction = DeclareLaunchArgument(
        "enable_interaction", default_value="true"
    )
    enable_behaviors = DeclareLaunchArgument("enable_behaviors", default_value="true")

    # Perception topics (forwarded to face_perception)
    image_topic = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/color/image_raw",
        description="RGB image topic",
    )
    depth_topic = DeclareLaunchArgument(
        "depth_topic",
        default_value="/camera/depth/image_rect_raw",
        description="Depth image topic",
    )
    camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/color/camera_info",
        description="CameraInfo topic",
    )

    # Base parameters (forwarded to base driver)
    base_cmd_vel_topic = DeclareLaunchArgument(
        "base_cmd_vel_topic",
        default_value="/cmd_vel",
        description="Twist command topic",
    )
    base_serial_port = DeclareLaunchArgument(
        "base_serial_port",
        default_value="/dev/ttyUSB0",
        description="Base motor controller serial port",
    )
    base_serial_baud = DeclareLaunchArgument(
        "base_serial_baud",
        default_value="115200",
        description="Base motor controller baudrate",
    )
    base_wheel_radius = DeclareLaunchArgument(
        "base_wheel_radius", default_value="0.033", description="Wheel radius (m)"
    )
    base_wheel_separation = DeclareLaunchArgument(
        "base_wheel_separation",
        default_value="0.16",
        description="Wheel separation (m)",
    )

    # Build launch description
    ld = LaunchDescription()

    # Declare common args
    ld.add_action(use_sim_time)
    ld.add_action(namespace)

    # Description args
    ld.add_action(start_description)
    ld.add_action(robot_name)
    ld.add_action(publish_joint_state)
    ld.add_action(use_joint_state_publisher_gui)

    # Sensors args
    ld.add_action(start_sensors)
    ld.add_action(lidar_type)
    ld.add_action(lidar_port)
    ld.add_action(lidar_baud)
    ld.add_action(lidar_frame_id)
    ld.add_action(camera_type)
    ld.add_action(camera_align_depth)
    ld.add_action(camera_pointcloud)

    # Core toggles
    ld.add_action(enable_base)
    ld.add_action(enable_memory)
    ld.add_action(enable_perception)
    ld.add_action(enable_emotion)
    ld.add_action(enable_display)
    ld.add_action(enable_interaction)
    ld.add_action(enable_behaviors)

    # Perception/base params
    ld.add_action(image_topic)
    ld.add_action(depth_topic)
    ld.add_action(camera_info_topic)
    ld.add_action(base_cmd_vel_topic)
    ld.add_action(base_serial_port)
    ld.add_action(base_serial_baud)
    ld.add_action(base_wheel_radius)
    ld.add_action(base_wheel_separation)

    # Includes (wrap in GroupAction to conditionally start)
    ld.add_action(
        GroupAction(
            actions=[_include_description()],
            condition=IfCondition(LaunchConfiguration("start_description")),
        )
    )

    ld.add_action(
        GroupAction(
            actions=[_include_sensors()],
            condition=IfCondition(LaunchConfiguration("start_sensors")),
        )
    )

    # Core stack (sensors disabled within core; provided by this bringup)
    ld.add_action(_include_core())

    return ld
