"""
IR Sensor Node
Publishes IR distance sensor readings
(Note: This is typically handled by ESP8266, this is a fallback local implementation)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header

try:
    import board
    import busio
    import adafruit_ads1x15.analog_in as AnalogIn
    from adafruit_ads1x15.ads1115 import ADS1115
    ADS1115_AVAILABLE = True
except ImportError:
    ADS1115_AVAILABLE = False


class IRSensorNode(Node):
    """
    Publishes IR distance sensor readings
    Supports ADS1115 I2C ADC for analog sensors
    
    Publications:
    - /sensor/ir_left (Range): Left IR distance
    - /sensor/ir_right (Range): Right IR distance
    """

    def __init__(self):
        super().__init__('ir_sensor_node')
        
        # Parameters
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('left_channel', 0)
        self.declare_parameter('right_channel', 1)
        self.declare_parameter('sensor_type', 'GP2Y0A21YK0F')  # Sharp IR sensor
        
        publish_rate = self.get_parameter('publish_rate').value
        self.left_channel = self.get_parameter('left_channel').value
        self.right_channel = self.get_parameter('right_channel').value
        self.sensor_type = self.get_parameter('sensor_type').value
        
        # Publishers
        self.left_pub = self.create_publisher(Range, '/sensor/ir_left', 10)
        self.right_pub = self.create_publisher(Range, '/sensor/ir_right', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_sensors)
        
        # Initialize ADC
        self.adc = None
        self.left_channel_obj = None
        self.right_channel_obj = None
        
        if ADS1115_AVAILABLE:
            self._init_adc()
        else:
            self.get_logger().warn(
                'ADS1115 not available. Install: pip install adafruit-circuitpython-ads1x15'
            )
        
        self.get_logger().info(f'IR Sensor node initialized. Rate: {publish_rate} Hz')

    def _init_adc(self):
        """Initialize ADS1115 ADC"""
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.adc = ADS1115(i2c)
            
            self.left_channel_obj = AnalogIn.AnalogIn(self.adc, self.left_channel)
            self.right_channel_obj = AnalogIn.AnalogIn(self.adc, self.right_channel)
            
            self.get_logger().info('ADS1115 ADC initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ADS1115: {e}')
            self.adc = None

    def _voltage_to_distance(self, voltage: float) -> float:
        """Convert voltage to distance based on sensor type"""
        if self.sensor_type == 'GP2Y0A21YK0F':
            # Sharp GP2Y0A21YK0F: distance ≈ 27.86 * V^(-1.15)
            # Typical range: 10-80cm
            if voltage > 0:
                distance = 27.86 * pow(voltage, -1.15)
            else:
                distance = 0.8  # Max range
            return min(distance / 100.0, 0.8)  # Convert to meters, clamp to max range
        else:
            # Generic: assume linear relationship
            return voltage * 0.2  # Example conversion

    def _publish_sensors(self):
        """Publish IR sensor readings"""
        if self.adc is None:
            return
        
        try:
            # Read left sensor
            left_voltage = self.left_channel_obj.voltage
            left_distance = self._voltage_to_distance(left_voltage)
            
            left_msg = Range()
            left_msg.header = Header()
            left_msg.header.stamp = self.get_clock().now().to_msg()
            left_msg.header.frame_id = 'ir_left_link'
            left_