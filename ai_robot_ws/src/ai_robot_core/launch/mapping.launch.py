import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ai_robot_core = get_package_share_directory('ai_robot_core')

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

    # Teleop
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ai_robot_core, 'launch', 'teleop.launch.py')
        )
    )

    # SLAM
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(pkg_ai_robot_core, 'config', 'slam_async.yaml')]
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_ai_robot_core, 'rviz', 'ai_robot.rviz')]
    )

    return LaunchDescription([
        gazebo,
        core_sensors,
        teleop,
        slam,
        rviz,
    ])
