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
    map_file = LaunchConfiguration('map_file')

    # --- Hardware Drivers ---
    
    # ESP8266 Bridge for motors and sensors
    esp8266_bridge_node = Node(
        package='ai_robot_hardware_drivers',
        executable='esp8266_bridge_node',
        name='esp8266_bridge',
        output='screen',
        # parameters=[os.path.join(drivers_pkg, 'config', 'your_config.yaml')] # Example
    )

    # Kinect Camera Driver
    kinect_node = Node(
        package='ai_robot_hardware_drivers',
        executable='kinect_interface_node',
        name='kinect_interface',
        output='screen'
    )

    # Local Audio Driver (TTS/STT)
    audio_node = Node(
        package='ai_robot_hardware_drivers',
        executable='audio_interface_node',
        name='audio_interface',
        output='screen'
    )

    # --- Core AI and Robot Logic ---

    # Robot State Publisher (uses URDF from core package)
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_pkg, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # AI Brain Nodes (from core package)
    face_recognition_node = Node(
        package='ai_robot_core',
        executable='face_recognition_node',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/camera/image_raw', '/camera/camera/image_raw'), # Remap to Kinect topic
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
             ('/robot_speech_output', '/robot/tts'), # Connect to audio node
        ]
    )

    person_follower_node = Node(
        package='ai_robot_core',
        executable='person_follower_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Navigation ---
    
    # Nav2 Bringup (from core package)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(core_pkg, 'config', 'nav2_params.yaml')
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
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
        navigation_launch
    ])
