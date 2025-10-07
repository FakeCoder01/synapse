#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Project Synapse — Nav2 Bringup (Step 3 deliverable)
#
# This launch file brings up the Navigation2 stack using the upstream
# nav2_bringup navigation_launch.py, with convenient arguments for:
# - map path or SLAM mode
# - params file
# - namespace
# - RViz 2 visualization
#
# Example usages:
#   # Use a saved map for localization + navigation:
#   ros2 launch synapse_navigation nav2_bringup.launch.py \
#     map:=/home/robot/maps/home_map.yaml
#
#   # Use SLAM (slam_toolbox) to build a map on the fly:
#   ros2 launch synapse_navigation nav2_bringup.launch.py slam:=true
#
#   # Override the default params and disable RViz:
#   ros2 launch synapse_navigation nav2_bringup.launch.py \
#     params_file:=/path/to/my_nav2_params.yaml use_rviz:=false
#
#   # Use a namespace:
#   ros2 launch synapse_navigation nav2_bringup.launch.py \
#     use_namespace:=true namespace:=synapse
#
# Notes:
# - If slam:=true, the map argument is ignored by nav2_bringup.
# - The default params file and rviz config are taken from nav2_bringup.
#
# License: Apache-2.0

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # Resolve upstream Nav2 bringup package paths for defaults
    nav2_share = get_package_share_directory("nav2_bringup")
    default_params = os.path.join(nav2_share, "params", "nav2_params.yaml")
    default_rviz = os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")
    nav2_launch = os.path.join(nav2_share, "launch", "navigation_launch.py")

    # Common launch arguments
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace for Nav2 nodes",
    )
    declare_use_namespace = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply the namespace to the Nav2 stack",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to a YAML map file to load (ignored if slam:=true)",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock (sim time) if true",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=TextSubstitution(text=default_params),
        description="Full path to the ROS2 parameters file for Nav2",
    )
    declare_slam = DeclareLaunchArgument(
        "slam",
        default_value="false",
        description="If true, start SLAM (slam_toolbox) instead of AMCL localization",
    )
    declare_use_composition = DeclareLaunchArgument(
        "use_composition",
        default_value="false",
        description="Use composed bringup in a container",
    )
    declare_container_name = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="Container name if use_composition is true",
    )
    declare_use_respawn = DeclareLaunchArgument(
        "use_respawn",
        default_value="false",
        description="Respawn Nav2 nodes on failure",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (e.g., debug, info, warn, error)",
    )

    # RViz toggles
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 with a default Nav2 view",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=TextSubstitution(text=default_rviz),
        description="RViz2 config file",
    )

    # Include upstream Nav2 bringup launch with our arguments
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file"),
            "autostart": LaunchConfiguration("autostart"),
            "slam": LaunchConfiguration("slam"),
            "use_composition": LaunchConfiguration("use_composition"),
            "container_name": LaunchConfiguration("container_name"),
            "use_respawn": LaunchConfiguration("use_respawn"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    # RViz2 node (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Group (namespace support is handled by the included Nav2 bringup)
    group = GroupAction(actions=[nav2_include, rviz_node])

    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_map)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_params_file)
    ld.add_action(declare_slam)
    ld.add_action(declare_use_composition)
    ld.add_action(declare_container_name)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_log_level)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_rviz_config)

    # Add actions
    ld.add_action(group)

    return ld
