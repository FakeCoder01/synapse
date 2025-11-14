# This launch file is for running navigation in the SIMULATION.
# It starts Gazebo, the robot, sensors, and then the Nav2 stack.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    core_pkg = get_package_share_directory('ai_robot_core')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(core_pkg, 'maps', 'my_map.yaml'),
            description='Full path to map file to load'
        ),

        # --- Simulation Environment ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(core_pkg, 'launch', 'gazebo.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(core_pkg, 'launch', 'core_sensors.launch.py')
            )
        ),

        # --- Navigation Stack ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(core_pkg, 'launch', 'nav2_bringup.launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(core_pkg, 'config', 'nav2_params.yaml')
            }.items()
        ),
    ])

