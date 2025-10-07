#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project Synapse — Perception Capture Launch
#
# Starts the capture_face_encoding_node, which provides a CaptureFaceEncoding
# service used by enrollment flows (e.g., initiated by the Conversation Manager).
#
# Example:
#   ros2 launch synapse_bringup perception_capture.launch.py
#
# Arguments:
#   - use_sim_time (bool):                Use /clock for time (default: false)
#   - namespace (string):                 Optional namespace for the node (default: "")
#   - default_image_topic (string):       Camera RGB topic to pre-subscribe (default: "/camera/color/image_raw")
#   - auto_subscribe_on_request (bool):   Auto-create subscriptions when requests target a different topic (default: true)
#
# The node exposes:
#   - Service: "~capture"
#   - Service: "/perception/capture_face_encoding"
#
# Notes:
# - For best results, run with a MultiThreadedExecutor. Most launch systems already
#   handle this for you. If using a single-threaded executor, having a default
#   subscription ensures frames are already buffered before service calls.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Launch-time arguments
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time from /clock",
    )
    namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace for the capture node",
    )
    default_image_topic = DeclareLaunchArgument(
        "default_image_topic",
        default_value="/camera/color/image_raw",
        description="Camera RGB image topic to pre-subscribe at startup",
    )
    auto_subscribe_on_request = DeclareLaunchArgument(
        "auto_subscribe_on_request",
        default_value="true",
        description="Auto-subscribe to requested topics when handling capture requests",
    )

    # LaunchConfigurations
    lc_use_sim_time = LaunchConfiguration("use_sim_time")
    lc_namespace = LaunchConfiguration("namespace")
    lc_default_image_topic = LaunchConfiguration("default_image_topic")
    lc_auto_subscribe_on_request = LaunchConfiguration("auto_subscribe_on_request")

    # Capture node
    capture_node = Node(
        package="synapse_perception",
        executable="capture_face_encoding_node",
        name="capture_face_encoding",
        namespace=lc_namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": lc_use_sim_time,
                "default_image_topic": lc_default_image_topic,
                "auto_subscribe_on_request": lc_auto_subscribe_on_request,
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time)
    ld.add_action(namespace)
    ld.add_action(default_image_topic)
    ld.add_action(auto_subscribe_on_request)
    ld.add_action(capture_node)
    return ld
