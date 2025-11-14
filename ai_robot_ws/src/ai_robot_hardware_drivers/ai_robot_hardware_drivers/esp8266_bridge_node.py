"""
ESP8266 Bridge Node
Bridges ROS2 and ESP8266 microcontroller via MQTT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import json
import paho.mqtt.client as mqtt
from threading import Lock
import time
import math

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class ESP8266Bridge(Node):
    """
    Bridges ROS2 topics with ESP8266 via MQTT
    
    Subscriptions:
    - /cmd_vel (Twist): Motor velocity commands
    
    Publications:
    - /odom (Odometry): Estimated odometry
    - /sensor/ir_left (Range): Left IR sensor distance
    - /robot/telemetry (String): Device telemetry
    - /robot/status (String): Device status
    """

    def __init__(self):
        super().__init__('esp8266_bridge')
        
        # Parameters
        self.declare_parameter('mqtt_host', '127.0.0.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_client_id', 'ros2_bridge')
        self.declare_parameter('mqtt_keepalive', 60)
        self.declare_parameter('wheel_separation', 0.2)
        self.declare_parameter('cmd_vel_timeout', 1.0)
        
        self.mqtt_host = self.get_parameter('mqtt_host').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.mqtt_client_id = self.get_parameter('mqtt_client_id').value
        self.mqtt_keepalive = self.get_parameter('mqtt_keepalive').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        
        # MQTT Topics
        self.mqtt_cmd_vel_topic = 'robot/cmd_vel'
        self.mqtt_motor_cmd_topic = 'robot/motor_cmd'
        self.mqtt_sensors_topic = 'robot/sensors'
        self.mqtt_status_topic = 'robot/status'
        self.mqtt_telemetry_topic = 'robot/telemetry'
        
        # ROS2 Subscriptions
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            qos_profile
        )
        
        # ROS2 Publishers
        self.ir_left_publisher = self.create_publisher(Range, '/sensor/ir_left', 10)
        self.ir_right_publisher = self.create_publisher(Range, '/sensor/ir_right', 10)
        self.telemetry_publisher = self.create_publisher(String, '/robot/telemetry', 10)
        self.status_publisher = self.create_publisher(String, '/robot/status', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # State
        self.mqtt_connected = False
        self.last_cmd_vel_time = time.time()
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.state_lock = Lock()

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_odom_time = self.get_clock().now()
        
        # MQTT Setup
        self.mqtt_client = mqtt.Client(self.mqtt_client_id)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        
        # Connect to MQTT
        self._connect_mqtt()
        
        # Timers
        self.mqtt_loop_timer = self.create_timer(0.1, self._mqtt_loop)
        self.cmd_vel_watchdog_timer = self.create_timer(0.1, self._cmd_vel_watchdog)
        self.odom_publish_timer = self.create_timer(0.05, self._publish_odometry) # 20 Hz

        self.get_logger().info(
            f'ESP8266 Bridge initialized. MQTT: {self.mqtt_host}:{self.mqtt_port}'
        )

    def _connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.mqtt_client.connect(
                self.mqtt_host,
                self.mqtt_port,
                self.mqtt_keepalive
            )
            self.mqtt_client.loop_start()
            self.get_logger().info('MQTT connection started')
        except Exception as e:
            self.get_logger().error(f'Failed to start MQTT: {e}')

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info('MQTT broker connected')
            
            # Subscribe to topics
            client.subscribe(self.mqtt_sensors_topic)
            client.subscribe(self.mqtt_status_topic)
            client.subscribe(self.mqtt_telemetry_topic)
            
            # Publish online status
            self._mqtt_publish(self.mqtt_status_topic, json.dumps({
                'status': 'connected',
                'node': 'ros2_bridge'
            }))
        else:
            self.get_logger().error(f'MQTT connection failed with code {rc}')
            self.mqtt_connected = False

    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        if rc != 0:
            self.get_logger().warn(f'Unexpected MQTT disconnection: {rc}')
        self.mqtt_connected = False

    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode())
            
            if topic == self.mqtt_sensors_topic:
                self._handle_sensor_data(payload)
            elif topic == self.mqtt_telemetry_topic:
                self._handle_telemetry(payload)
            elif topic == self.mqtt_status_topic:
                self._handle_status(payload)
        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')

    def _handle_sensor_data(self, payload):
        """Process sensor data from ESP8266"""
        try:
            # Parse IR sensor data
            if 'ir_left_cm' in payload:
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'ir_left_link'
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = 0.5  # radians
                msg.min_range = 0.2
                msg.max_range = 0.8
                msg.range = payload['ir_left_cm'] / 100.0  # Convert cm to m
                self.ir_left_publisher.publish(msg)
            
            if 'ir_right_cm' in payload:
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'ir_right_link'
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = 0.5
                msg.min_range = 0.2
                msg.max_range = 0.8
                msg.range = payload['ir_right_cm'] / 100.0
                self.ir_right_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error handling sensor data: {e}')

    def _handle_telemetry(self, payload):
        """Process telemetry from ESP8266"""
        msg = String()
        msg.data = json.dumps(payload)
        self.telemetry_publisher.publish(msg)

    def _handle_status(self, payload):
        """Process status from ESP8266"""
        msg = String()
        msg.data = json.dumps(payload)
        self.status_publisher.publish(msg)

    def _cmd_vel_callback(self, msg: Twist):
        """Process velocity commands from Nav2 or teleop"""
        with self.state_lock:
            self.last_linear_x = msg.linear.x
            self.last_angular_z = msg.angular.z
            self.last_cmd_vel_time = time.time()
        
        # Convert Twist to motor commands using differential drive kinematics
        cmd_json = self._twist_to_motor_cmd(msg)
        
        # Publish to ESP8266 via MQTT
        self._mqtt_publish(self.mqtt_motor_cmd_topic, json.dumps(cmd_json))

    def _twist_to_motor_cmd(self, msg: Twist) -> dict:
        """
        Convert geometry_msgs/Twist to motor commands
        Uses differential drive kinematics
        """
        linear_x = msg.linear.x  # Forward/backward
        angular_z = msg.angular.z  # Rotation
        
        # Differential drive: each wheel gets different velocity
        left_vel = linear_x - angular_z * (self.wheel_separation / 2.0)
        right_vel = linear_x + angular_z * (self.wheel_separation / 2.0)
        
        # Normalize if needed
        max_vel = max(abs(left_vel), abs(right_vel))
        if max_vel > 1.0:
            left_vel /= max_vel
            right_vel /= max_vel
        
        # Convert to PWM values (0-255, with sign for direction)
        left_pwm = int(left_vel * 255)
        right_pwm = int(right_vel * 255)
        
        # All 4 motors: front and back pairs move together
        return {
            'fl': left_pwm,   # Front-Left
            'fr': right_pwm,  # Front-Right
            'bl': left_pwm,   # Back-Left
            'br': right_pwm   # Back-Right
        }

    def _cmd_vel_watchdog(self):
        """Watchdog to stop motors if cmd_vel stops arriving"""
        current_time = time.time()
        with self.state_lock:
            time_since_last_cmd = current_time - self.last_cmd_vel_time
            
            if time_since_last_cmd > self.cmd_vel_timeout:
                if self.last_linear_x != 0.0 or self.last_angular_z != 0.0:
                    # Send stop command
                    self._mqtt_publish(
                        self.mqtt_motor_cmd_topic,
                        json.dumps({'fl': 0, 'fr': 0, 'bl': 0, 'br': 0})
                    )
                    self.get_logger().warn('cmd_vel timeout: stopping motors')
                    self.last_linear_x = 0.0
                    self.last_angular_z = 0.0
    
    def _publish_odometry(self):
        """Calculate and publish odometry based on last command"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time

        with self.state_lock:
            vx = self.last_linear_x
            vth = self.last_angular_z

        delta_x = vx * math.cos(self.th) * dt
        delta_y = vx * math.sin(self.th) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Create and publish TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = t.transform.rotation
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.th / 2.0)
        q.w = math.cos(self.th / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

        # Create and publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        self.odom_publisher.publish(odom)

    def _mqtt_loop(self):
        """Handle MQTT loop"""
        if self.mqtt_connected:
            # MQTT client loop is running in background thread
            pass
        else:
            # Try to reconnect
            try:
                if not self.mqtt_client.is_connected():
                    self.mqtt_client.reconnect()
            except Exception as e:
                self.get_logger().debug(f'MQTT reconnect attempt failed: {e}')

    def _mqtt_publish(self, topic: str, payload: str):
        """Publish message to MQTT"""
        try:
            if self.mqtt_connected:
                self.mqtt_client.publish(topic, payload)
        except Exception as e:
            self.get_logger().error(f'MQTT publish failed: {e}')

    def destroy_node(self):
        """Cleanup on node shutdown"""
        if self.mqtt_client.is_connected():
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    bridge = ESP8266Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()