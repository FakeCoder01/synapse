import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ai_robot_core = get_package_share_directory('ai_robot_core')

    # Depth to scan
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        remappings=[('depth', '/camera/depth/image_raw'),
                    ('depth_camera_info', '/camera/depth/camera_info')],
        parameters=[os.path.join(pkg_ai_robot_core, 'config', 'depth_to_scan.yaml')]
    )

    return LaunchDescription([
        depth_to_scan,
    ])
