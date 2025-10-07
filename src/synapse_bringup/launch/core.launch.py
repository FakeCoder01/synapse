#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project Synapse — Core System Bringup
#
# Composable launch to start the "core" stack:
# - Base driver (diff-drive)
# - Memory server (SQLite + Vector DB)
# - Perception (face detection/recognition)
# - Emotion engine
# - Face/eyes display
# - Conversation manager (STT → LLM → TTS)
# - Behaviors (Patrol, Seek-and-Greet, Follow)
# - Optional sensors bringup (LiDAR + Camera) via sensors.launch.py
#
# Example:
#   ros2 launch synapse_bringup core.launch.py
#
# Toggle modules:
#   ros2 launch synapse_bringup core.launch.py enable_behaviors:=false
#
# Include sensors (defaults to true). Forward sensor arguments:
#   ros2 launch synapse_bringup core.launch.py \
#     start_sensors:=true \
#     lidar_type:=rplidar lidar_port:=/dev/ttyUSB0 lidar_baud:=115200 lidar_frame_id:=laser \
#     camera_type:=realsense camera_align_depth:=true camera_pointcloud:=true
#
# License: Apache-2.0

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def _sensors_include():
    """Include the sensors.launch.py with pass-through arguments."""
    bringup_share = get_package_share_directory("synapse_bringup")
    sensors_launch = os.path.join(bringup_share, "launch", "sensors.launch.py")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch),
        launch_arguments={
            # Direct pass-through from LaunchConfigurations:
            "lidar_type": LaunchConfiguration("lidar_type"),
            "lidar_port": LaunchConfiguration("lidar_port"),
            "lidar_baud": LaunchConfiguration("lidar_baud"),
            "lidar_frame_id": LaunchConfiguration("lidar_frame_id"),
            "camera_type": LaunchConfiguration("camera_type"),
            "camera_align_depth": LaunchConfiguration("camera_align_depth"),
            "camera_pointcloud": LaunchConfiguration("camera_pointcloud"),
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
        "namespace",
        default_value="",
        description="Optional top-level namespace for all core nodes",
    )

    # Module toggles
    enable_base = DeclareLaunchArgument("enable_base", default_value="true")
    enable_memory = DeclareLaunchArgument("enable_memory", default_value="true")
    enable_perception = DeclareLaunchArgument("enable_perception", default_value="true")
    enable_emotion = DeclareLaunchArgument("enable_emotion", default_value="true")
    enable_display = DeclareLaunchArgument("enable_display", default_value="true")
    enable_interaction = DeclareLaunchArgument(
        "enable_interaction", default_value="true"
    )
    enable_behaviors = DeclareLaunchArgument("enable_behaviors", default_value="true")

    # Sensors toggle and arguments (forwarded to sensors.launch.py)
    start_sensors = DeclareLaunchArgument("start_sensors", default_value="true")
    lidar_type = DeclareLaunchArgument(
        "lidar_type", default_value="rplidar", description="rplidar | hokuyo | none"
    )
    lidar_port = DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0")
    lidar_baud = DeclareLaunchArgument("lidar_baud", default_value="115200")
    lidar_frame_id = DeclareLaunchArgument("lidar_frame_id", default_value="laser")

    camera_type = DeclareLaunchArgument(
        "camera_type", default_value="realsense", description="realsense | none"
    )
    camera_align_depth = DeclareLaunchArgument(
        "camera_align_depth", default_value="true"
    )
    camera_pointcloud = DeclareLaunchArgument("camera_pointcloud", default_value="true")

    # Perception topic parameters
    image_topic = DeclareLaunchArgument(
        "image_topic", default_value="/camera/color/image_raw"
    )
    depth_topic = DeclareLaunchArgument(
        "depth_topic", default_value="/camera/depth/image_rect_raw"
    )
    camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera/color/camera_info"
    )

    # Base driver parameters (typical diff-drive defaults; override as needed)
    base_cmd_vel = DeclareLaunchArgument("base_cmd_vel_topic", default_value="/cmd_vel")
    base_serial_port = DeclareLaunchArgument(
        "base_serial_port", default_value="/dev/ttyUSB0"
    )
    base_serial_baud = DeclareLaunchArgument("base_serial_baud", default_value="115200")
    base_wheel_radius = DeclareLaunchArgument(
        "base_wheel_radius", default_value="0.033"
    )
    base_wheel_separation = DeclareLaunchArgument(
        "base_wheel_separation", default_value="0.16"
    )

    # Nodes
    base_node = Node(
        package="synapse_base",
        executable="base_driver_node",
        name="base_driver_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "cmd_vel_topic": LaunchConfiguration("base_cmd_vel_topic"),
                "serial_port": LaunchConfiguration("base_serial_port"),
                "serial_baud": LaunchConfiguration("base_serial_baud"),
                "wheel_radius": LaunchConfiguration("base_wheel_radius"),
                "wheel_separation": LaunchConfiguration("base_wheel_separation"),
                # Additional defaults; can be overridden by a params file if desired
                "publish_odom": True,
                "publish_tf": True,
                "odom_source": "command",
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_base")),
    )

    memory_node = Node(
        package="synapse_memory",
        executable="memory_server",
        name="memory_server",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # Paths can be overridden via params or environment.
                # 'base_dir': '/home/robot/.synapse',
                # 'sqlite_path': '',
                # 'chroma_path': '',
                # 'embed_model_name': 'all-MiniLM-L6-v2',
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_memory")),
    )

    perception_node = Node(
        package="synapse_perception",
        executable="face_perception_node",
        name="face_perception_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                # Tuning (override as needed)
                "detection_model": "hog",  # hog | cnn
                "upsample_times": 1,
                "encoding_model": "small",  # small | large
                "match_tolerance": 0.48,
                "top_k": 3,
                "primary_strategy": "closest",  # closest | center | largest
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_perception")),
    )

    emotion_node = Node(
        package="synapse_emotion",
        executable="emotion_engine",
        name="emotion_engine",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # Tuning params available; see package README/code for details
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_emotion")),
    )

    display_node = Node(
        package="synapse_display",
        executable="face_display_node",
        name="face_display_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # 'use_color': True,
                # 'fullscreen': True,
                # 'show_debug': False,
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_display")),
    )

    interaction_node = Node(
        package="synapse_interaction",
        executable="conversation_manager",
        name="conversation_manager",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # STT/TTS/LLM provider selection via parameters or environment variables.
                # See README for required API keys and optional Vosk model path, etc.
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_interaction")),
    )

    behaviors_node = Node(
        package="synapse_behaviors",
        executable="behavior_manager",
        name="behavior_manager",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # Waypoints and behavior parameters can be set via YAML or CLI.
            }
        ],
        condition=IfCondition(LaunchConfiguration("enable_behaviors")),
    )

    # Sensors group (conditional include)
    sensors_group = GroupAction(
        actions=[_sensors_include()],
        condition=IfCondition(LaunchConfiguration("start_sensors")),
    )

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(use_sim_time)
    ld.add_action(namespace)

    ld.add_action(enable_base)
    ld.add_action(enable_memory)
    ld.add_action(enable_perception)
    ld.add_action(enable_emotion)
    ld.add_action(enable_display)
    ld.add_action(enable_interaction)
    ld.add_action(enable_behaviors)

    ld.add_action(start_sensors)
    ld.add_action(lidar_type)
    ld.add_action(lidar_port)
    ld.add_action(lidar_baud)
    ld.add_action(lidar_frame_id)
    ld.add_action(camera_type)
    ld.add_action(camera_align_depth)
    ld.add_action(camera_pointcloud)

    ld.add_action(image_topic)
    ld.add_action(depth_topic)
    ld.add_action(camera_info_topic)

    ld.add_action(base_cmd_vel)
    ld.add_action(base_serial_port)
    ld.add_action(base_serial_baud)
    ld.add_action(base_wheel_radius)
    ld.add_action(base_wheel_separation)

    # Add actions (order: sensors first, then core nodes)
    ld.add_action(sensors_group)

    ld.add_action(base_node)
    ld.add_action(memory_node)
    ld.add_action(perception_node)
    ld.add_action(emotion_node)
    ld.add_action(display_node)
    ld.add_action(interaction_node)
    ld.add_action(behaviors_node)

    return ld
