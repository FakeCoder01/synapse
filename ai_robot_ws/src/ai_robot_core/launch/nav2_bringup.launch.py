# This launch file is a centralized place to launch the Nav2 stack.
# It is intended to be included by other launch files.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    core_pkg = get_package_share_directory('ai_robot_core')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(core_pkg, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameters file'
        ),

        # Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(core_pkg, 'rviz', 'ai_robot.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
