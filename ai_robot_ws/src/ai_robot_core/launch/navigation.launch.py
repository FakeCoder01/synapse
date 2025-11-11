import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ai_robot_core = get_package_share_directory('ai_robot_core')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    map = LaunchConfiguration('map')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ai_robot_core, 'launch', 'gazebo.launch.py')
        )
    )

    # Core sensors
    core_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ai_robot_core, 'launch', 'core_sensors.launch.py')
        )
    )

    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map,
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_ai_robot_core, 'config', 'nav2_params.yaml')
        }.items()
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_ai_robot_core, 'rviz', 'ai_robot.rviz')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_ai_robot_core, 'maps', 'my_map.yaml'),
            description='Full path to map file to load'),
        gazebo,
        core_sensors,
        nav2,
        rviz,
    ])
