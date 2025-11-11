import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ai_robot_core = get_package_share_directory('ai_robot_core')
    map = LaunchConfiguration('map')

    # Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ai_robot_core, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'map': map}.items()
    )

    # Brain
    face_recognition_node = Node(
        package='ai_robot_core',
        executable='face_recognition_node',
        name='face_recognition_node',
        output='screen'
    )

    memory_manager_node = Node(
        package='ai_robot_core',
        executable='memory_manager_node',
        name='memory_manager_node',
        output='screen'
    )

    emotion_engine_node = Node(
        package='ai_robot_core',
        executable='emotion_engine_node',
        name='emotion_engine_node',
        output='screen'
    )

    interaction_manager_node = Node(
        package='ai_robot_core',
        executable='interaction_manager_node',
        name='interaction_manager_node',
        output='screen'
    )

    person_follower_node = Node(
        package='ai_robot_core',
        executable='person_follower_node',
        name='person_follower_node',
        output='screen'
    )

    # UI
    rqt_ui = ExecuteProcess(
        cmd=['rqt', '--force-discover', '-p', 'ai_robot_core.ui.robot_ui_plugin.RobotUIPlugin'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_ai_robot_core, 'maps', 'my_map.yaml'),
            description='Full path to map file to load'),
        navigation,
        face_recognition_node,
        memory_manager_node,
        emotion_engine_node,
        interaction_manager_node,
        person_follower_node,
        rqt_ui,
    ])
