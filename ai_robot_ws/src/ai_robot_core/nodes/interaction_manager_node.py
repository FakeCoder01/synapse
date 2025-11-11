import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ai_robot_msgs.msg import PersonArray
from ai_robot_msgs.srv import GetConversation, StoreConversation
import datetime

class InteractionManagerNode(Node):
    def __init__(self):
        super().__init__('interaction_manager_node')
        self.recently_greeted_ids = {} # person_id -> last_greeted_time

        self.detected_people_subscription = self.create_subscription(
            PersonArray,
            '/detected_people',
            self.detected_people_callback,
            10)
        
        self.get_conversation_client = self.create_client(GetConversation, '/get_conversation')
        self.store_conversation_client = self.create_client(StoreConversation, '/store_conversation')
        
        self.robot_speech_output_publisher = self.create_publisher(String, '/robot_speech_output', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def detected_people_callback(self, msg):
        if not msg.people:
            return

        # Find the largest person in the frame
        largest_person = max(msg.people, key=lambda p: (p.bounding_box[2] - p.bounding_box[0]) * (p.bounding_box[3] - p.bounding_box[1]))

        now = datetime.datetime.now()
        
        if largest_person.person_id not in self.recently_greeted_ids or (now - self.recently_greeted_ids[largest_person.person_id]).total_seconds() > 300:
            self.recently_greeted_ids[largest_person.person_id] = now
            
            if largest_person.is_known:
                self.greet_known_person(largest_person)
            else:
                self.greet_unknown_person(largest_person)

    def greet_known_person(self, person):
        req = GetConversation.Request()
        req.person_id = person.person_id
        req.line_count = 1
        
        future = self.get_conversation_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        last_line = "we haven't talked before"
        if future.result() and future.result().history:
            last_line = future.result().history[0]

        greeting = f"Hello, {person.name}! Good to see you. Last time we talked about {last_line}."
        self.publish_speech(greeting)
        self.save_line(person.person_id, "robot", greeting)

    def greet_unknown_person(self, person):
        # Turn to face the person
        twist = Twist()
        image_center_x = 320 # a 640x480 image
        person_center_x = person.bounding_box[0] + (person.bounding_box[2] - person.bounding_box[0]) / 2
        angular_error = image_center_x - person_center_x
        twist.angular.z = -0.001 * angular_error
        self.cmd_vel_publisher.publish(twist)

        greeting = "Hello! I don't believe we've met."
        self.publish_speech(greeting)
        self.save_line(person.person_id, "robot", greeting)

    def publish_speech(self, text):
        speech_msg = String()
        speech_msg.data = text
        self.robot_speech_output_publisher.publish(speech_msg)
        self.get_logger().info(f"Saying: {text}")

    def save_line(self, person_id, speaker, line):
        req = StoreConversation.Request()
        req.person_id = person_id
        req.speaker = speaker
        req.line = line
        self.store_conversation_client.call_async(req)

    def call_llm_api(self, history, new_line):
        # This is a stub for the user to replace with their own LLM API call
        # import requests
        # response = requests.post("https://api.llm.com/chat", json={"history": history, "new_line": new_line})
        # return response.json()["text"]
        return f"That's interesting. Can you tell me more?"


def main(args=None):
    rclpy.init(args=args)
    node = InteractionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
