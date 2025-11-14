"""
Kinect Interface Node
Publishes color and depth data from Xbox Kinect camera as ROS2 messages
"""

import rclpy
from rclpy.node import Node
import threading
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import time

try:
    import pykinect2
    from pykinect2 import PyKinectRuntime, FrameSourceTypes
    KINECT_AVAILABLE = True
except ImportError:
    KINECT_AVAILABLE = False


class KinectInterfaceNode(Node):
    """
    Publishes Kinect color and depth frames as ROS2 Image messages
    
    Publications:
    - /camera/camera/image_raw (sensor_msgs/Image): RGB image
    - /camera/camera/camera_info (sensor_msgs/CameraInfo): RGB camera info
    - /camera/depth/image_raw (sensor_msgs/Image): Depth image
    - /camera/depth/camera_info (sensor_msgs/CameraInfo): Depth camera info
    """

    def __init__(self):
        super().__init__('kinect_interface')
        
        # Parameters
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_depth', True)
        
        self.frame_rate = self.get_parameter('frame_rate').value
        self.enable_color = self.get_parameter('enable_color').value
        self.enable_depth = self.get_parameter('enable_depth').value
        
        if not KINECT_AVAILABLE:
            self.get_logger().error(
                'PyKinect2 not available. Install: pip install pykinect2'
            )
            return
        
        # Initialize Kinect
        self._init_kinect()
        
        # Publishers
        if self.enable_color:
            self.color_image_pub = self.create_publisher(
                Image, '/camera/camera/image_raw', 10
            )
            self.color_info_pub = self.create_publisher(
                CameraInfo, '/camera/camera/camera_info', 10
            )
        
        if self.enable_depth:
            self.depth_image_pub = self.create_publisher(
                Image, '/camera/depth/image_raw', 10
            )
            self.depth_info_pub = self.create_publisher(
                CameraInfo, '/camera/depth/camera_info', 10
            )
        
        # Frame loop
        frame_interval = 1.0 / self.frame_rate
        self.timer = self.create_timer(frame_interval, self._publish_frames)
        
        # Thread for frame publishing
        self.running = True
        
        self.get_logger().info(
            f'Kinect Interface initialized. Frame rate: {self.frame_rate} Hz'
        )

    def _init_kinect(self):
        """Initialize Kinect sensor"""
        try:
            frame_sources = 0
            if self.enable_color:
                frame_sources |= FrameSourceTypes.Color
            if self.enable_depth:
                frame_sources |= FrameSourceTypes.Depth
            
            self.kinect = PyKinectRuntime(frame_sources)
            self.color_frame_width = self.kinect.color_frame_desc.Width
            self.color_frame_height = self.kinect.color_frame_desc.Height
            self.depth_frame_width = self.kinect.depth_frame_desc.Width
            self.depth_frame_height = self.kinect.depth_frame_desc.Height
            
            self.get_logger().info(
                f'Kinect initialized. Color: {self.color_frame_width}x{self.color_frame_height}, '
                f'Depth: {self.depth_frame_width}x{self.depth_frame_height}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Kinect: {e}')
            self.kinect = None

    def _publish_frames(self):
        """Publish color and depth frames"""
        if self.kinect is None:
            return
        
        try:
            # Publish color frame
            if self.enable_color and self.kinect.has_new_color_frame():
                self._publish_color_frame()
            
            # Publish depth frame
            if self.enable_depth and self.kinect.has_new_depth_frame():
                self._publish_depth_frame()
        except Exception as e:
            self.get_logger().error(f'Error publishing frames: {e}')

    def _publish_color_frame(self):
        """Publish color image"""
        try:
            frame = self.kinect.get_last_color_frame()
            
            # Reshape to image format (BGRA)
            frame = frame.reshape((self.color_frame_height, self.color_frame_width, 4))
            
            # Convert BGRA to BGR for OpenCV compatibility
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            # Create ROS Image message
            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_frame'
            msg.height = self.color_frame_height
            msg.width = self.color_frame_width
            msg.encoding = 'bgr8'
            msg.is_bigendian = False
            msg.step = self.color_frame_width * 3
            msg.data = frame_bgr.tobytes()
            
            self.color_image_pub.publish(msg)
            
            # Publish camera info
            info_msg = self._get_color_camera_info()
            info_msg.header = msg.header
            self.color_info_pub.publish(info_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing color frame: {e}')

    def _publish_depth_frame(self):
        """Publish depth image"""
        try:
            frame = self.kinect.get_last_depth_frame()
            
            # Reshape to image format (uint16)
            frame = frame.reshape((self.depth_frame_height, self.depth_frame_width))
            
            # Create ROS Image message
            msg = Image()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_depth_frame'
            msg.height = self.depth_frame_height
            msg.width = self.depth_frame_width
            msg.encoding = 'mono16'
            msg.is_bigendian = False
            msg.step = self.depth_frame_width * 2
            msg.data = frame.tobytes()
            
            self.depth_image_pub.publish(msg)
            
            # Publish camera info
            info_msg = self._get_depth_camera_info()
            info_msg.header = msg.header
            self.depth_info_pub.publish(info_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing depth frame: {e}')

    def _get_color_camera_info(self) -> CameraInfo:
        """Get color camera calibration info"""
        info = CameraInfo()
        info.height = self.color_frame_height
        info.width = self.color_frame_width
        
        # Kinect v2 intrinsic parameters (approximate)
        # fx = fy = 366.5
        # cx = 256, cy = 212
        info.k = [366.5, 0.0, 256.0,
                  0.0, 366.5, 212.0,
                  0.0, 0.0, 1.0]
        info.p = [366.5, 0.0, 256.0, 0.0,
                  0.0, 366.5, 212.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        
        return info

    def _get_depth_camera_info(self) -> CameraInfo:
        """Get depth camera calibration info"""
        info = CameraInfo()
        info.height = self.depth_frame_height
        info.width = self.depth_frame_width
        
        # Kinect v2 depth intrinsic parameters (approximate)
        info.k = [360.0, 0.0, 256.0,
                  0.0, 360.0, 212.0,
                  0.0, 0.0, 1.0]
        info.p = [360.0, 0.0, 256.0, 0.0,
                  0.0, 360.0, 212.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        
        return info

    def destroy_node(self):
        """Cleanup"""
        self.running = False
        if hasattr(self, 'kinect') and self.kinect is not None:
            self.kinect = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinectInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()