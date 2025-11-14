import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    core_pkg = get_package_share_directory('ai_robot_core')
    drivers_pkg = get_package_share_directory('ai_robot_hardware_drivers')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file', default=os.path.join(core_pkg, 'config', 'nav2_params.yaml'))
    hardware_params_file = LaunchConfiguration('hardware_params_file', default=os.path.join(drivers_pkg, 'config', 'hardware_params.yaml'))

    # --- Hardware Drivers ---
    
    esp8266_bridge_node = Node(
        package='ai_robot_hardware_drivers',
        executable='esp8266_bridge_node',
        name='esp8266_bridge',
        output='screen',
        parameters=[hardware_params_file]
    )

    kinect_node = Node(
        package='ai_robot_hardware_drivers',
        executable='kinect_interface_node',
        name='kinect_interface',
        output='screen'
    )

    audio_node = Node(
        package='ai_robot_hardware_drivers',
        executable='audio_interface_node',
        name='audio_interface',
        output='screen'
    )

    # --- Core AI and Robot Logic ---

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_pkg, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    face_recognition_node = Node(
        package='ai_robot_core',
        executable='face_recognition_node',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/camera/image_raw', '/camera/camera/image_raw'),
        ]
    )
    
    memory_manager_node = Node(
        package='ai_robot_core',
        executable='memory_manager_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    emotion_engine_node = Node(
        package='ai_robot_core',
        executable='emotion_engine_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    interaction_manager_node = Node(
        package='ai_robot_core',
        executable='interaction_manager_node',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
             ('/robot_speech_output', '/robot/tts'),
        ]
    )

    person_follower_node = Node(
        package='ai_robot_core',
        executable='person_follower_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Navigation ---
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_pkg, 'launch', 'nav2_bringup.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Full path to the map file to load for navigation.'
        ),
        
        # Drivers
        esp8266_bridge_node,
        kinect_node,
        audio_node,
        
        # Core
        robot_state_publisher_launch,
        face_recognition_node,
        memory_manager_node,
        emotion_engine_node,
        interaction_manager_node,
        person_follower_node,
        
        # Navigation
        nav2_launch
    ])
