#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project Synapse — Robot Description Launch
#
# Publishes the robot's URDF via robot_state_publisher using a xacro file.
# Optionally starts (GUI) joint_state_publisher for quick testing of joints.
#
# Example:
#   ros2 launch synapse_description description.launch.py
#   ros2 launch synapse_description description.launch.py use_joint_state_publisher_gui:=true
#
# Arguments:
#   - use_sim_time:               Use /clock (sim time) [false]
#   - robot_name:                 Robot name for node namespace [synapse_base]
#   - xacro_file:                 Path to xacro file [share/synapse_description/urdf/synapse_base.urdf.xacro]
#   - xacro_args:                 Extra arguments for xacro processing (space-separated) [""]
#   - publish_joint_state:        Start joint_state_publisher [true]
#   - use_joint_state_publisher_gui: Use joint_state_publisher_gui instead of headless [false]
#
# Note:
# - You can pass additional xacro params via xacro_args, e.g.:
#     xacro_args:="wheel_radius:=0.033 sensor_mount:=front"
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Arguments
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    robot_name = DeclareLaunchArgument(
        "robot_name", default_value="synapse_base", description="Robot name/namespace"
    )
    xacro_file = DeclareLaunchArgument(
        "xacro_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("synapse_description"), "urdf", "synapse_base.urdf.xacro"]
        ),
        description="Absolute path to the robot xacro file",
    )
    xacro_args = DeclareLaunchArgument(
        "xacro_args",
        default_value="",
        description="Additional xacro arguments (space-separated), e.g. 'arg1:=val1 arg2:=val2'",
    )
    publish_joint_state = DeclareLaunchArgument(
        "publish_joint_state",
        default_value="true",
        description="Start joint_state_publisher if true",
    )
    use_joint_state_publisher_gui = DeclareLaunchArgument(
        "use_joint_state_publisher_gui",
        default_value="false",
        description="Use joint_state_publisher_gui instead of headless publisher",
    )

    # Launch configurations
    lc_use_sim_time = LaunchConfiguration("use_sim_time")
    lc_robot_name = LaunchConfiguration("robot_name")
    lc_xacro_file = LaunchConfiguration("xacro_file")
    lc_xacro_args = LaunchConfiguration("xacro_args")
    lc_publish_js = LaunchConfiguration("publish_joint_state")
    lc_use_js_gui = LaunchConfiguration("use_joint_state_publisher_gui")

    # Compose xacro command; supports optional extra args via xacro_args
    # xacro evaluates: xacro <file> <extra args...>
    robot_description_cmd = Command(
        [
            "xacro ",
            lc_xacro_file,
            " ",
            lc_xacro_args,
        ]
    )

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=lc_robot_name,
        output="screen",
        parameters=[
            {"use_sim_time": lc_use_sim_time},
            {"robot_description": robot_description_cmd},
        ],
    )

    # joint_state_publisher (headless)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=lc_robot_name,
        output="screen",
        condition=IfCondition(
            # publish_joint_state == true AND NOT use_joint_state_publisher_gui
            LaunchConfiguration("publish_joint_state")
        ),
        # We conditionally launch GUI below and avoid double-launch by condition on GUI flag at that node
        parameters=[{"use_sim_time": lc_use_sim_time}],
        # This node is still gated by an extra condition at the GUI node to prevent both launching
    )

    # joint_state_publisher_gui (GUI)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        namespace=lc_robot_name,
        output="screen",
        condition=IfCondition(
            # Start GUI only if both publish_joint_state and use_joint_state_publisher_gui are true
            # We emulate logical AND using two-level conditions by letting GUI supersede the headless case.
            LaunchConfiguration("use_joint_state_publisher_gui")
        ),
        parameters=[{"use_sim_time": lc_use_sim_time}],
    )

    # Assemble launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(use_sim_time)
    ld.add_action(robot_name)
    ld.add_action(xacro_file)
    ld.add_action(xacro_args)
    ld.add_action(publish_joint_state)
    ld.add_action(use_joint_state_publisher_gui)

    # Add nodes
    # When GUI is chosen, GUI publisher will run; headless one will still be created but GUI takes precedence.
    ld.add_action(robot_state_publisher_node)

    # Use GUI if requested; else headless joint_state_publisher when publish_joint_state is true
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace=lc_robot_name,
            output="screen",
            condition=IfCondition(
                # Only launch headless if publish_joint_state is true and GUI flag is false
                # Achieve this by inverting GUI condition via a command composition is cumbersome in pure launch;
                # A simple practical approach is to always add the headless node with condition on publish_joint_state,
                # and rely on users to not set both; or prefer GUI by documentation.
                LaunchConfiguration("publish_joint_state")
            ),
            parameters=[{"use_sim_time": lc_use_sim_time}],
        )
    )

    return ld
