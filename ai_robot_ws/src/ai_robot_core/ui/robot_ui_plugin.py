import rclpy
from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QPixmap, QImage
from python_qt_binding import loadUi
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import os
from ament_index_python.packages import get_package_share_directory

class RobotUIPlugin(Plugin):
    def __init__(self, context):
        super(RobotUIPlugin, self).__init__(context)
        self.setObjectName('RobotUIPlugin')

        self._widget = QWidget()
        ui_file = os.path.join(get_package_share_directory('ai_robot_core'), 'ui', 'robot_ui.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RobotUI')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self.bridge = CvBridge()

        # This is a bit of a hack to get a node into a rqt plugin
        self.node = context.node
        self.image_subscription = self.node.create_subscription(
            Image,
            '/annotated_image',
            self.image_callback,
            10)
        self.emotion_subscription = self.node.create_subscription(
            String,
            '/robot_emotion',
            self.emotion_callback,
            10)
        
        self._widget.video_feed_label.setText("Waiting for video feed...")
        self._widget.emotion_label.setText("😐")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
            pixmap = QPixmap.fromImage(q_image)
            self._widget.video_feed_label.setPixmap(pixmap)
        except Exception as e:
            self.node.get_logger().error(f"Failed to process image: {e}")

    def emotion_callback(self, msg):
        emotion = msg.data
        emoji = "😐"
        if emotion == "happy":
            emoji = "🙂"
        elif emotion == "curious":
            emoji = "🤔"
        self._widget.emotion_label.setText(emoji)

    def shutdown_plugin(self):
        self.image_subscription.destroy()
        self.emotion_subscription.destroy()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
