import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_robot_msgs.msg import PersonArray

class EmotionEngineNode(Node):
    def __init__(self):
        super().__init__('emotion_engine_node')
        self.current_emotion = "neutral"
        self.last_person_seen_time = self.get_clock().now()

        self.detected_people_subscription = self.create_subscription(
            PersonArray,
            '/detected_people',
            self.detected_people_callback,
            10)
        
        self.robot_emotion_publisher = self.create_publisher(String, '/robot_emotion', 10)
        
        self.timer = self.create_timer(1.0, self.check_for_neutral)

    def detected_people_callback(self, msg):
        self.last_person_seen_time = self.get_clock().now()
        
        if not msg.people:
            return

        known_person_detected = any(person.is_known for person in msg.people)
        
        if known_person_detected:
            self.set_emotion("happy")
        else:
            self.set_emotion("curious")

    def check_for_neutral(self):
        if (self.get_clock().now() - self.last_person_seen_time).nanoseconds / 1e9 > 10:
            self.set_emotion("neutral")

    def set_emotion(self, new_emotion):
        if self.current_emotion != new_emotion:
            self.current_emotion = new_emotion
            emotion_msg = String()
            emotion_msg.data = self.current_emotion
            self.robot_emotion_publisher.publish(emotion_msg)
            self.get_logger().info(f"Emotion changed to: {self.current_emotion}")

def main(args=None):
    rclpy.init(args=args)
    node = EmotionEngineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
