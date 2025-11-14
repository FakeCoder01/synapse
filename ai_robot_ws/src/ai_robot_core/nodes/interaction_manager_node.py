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
        
        self.audio_transcription_subscription = self.create_subscription(
            String,
            '/robot/audio_transcription',
            self.audio_transcription_callback,
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

    def audio_transcription_callback(self, msg):
        """Handles transcribed speech from the user."""
        input_text = msg.data
        self.get_logger().info(f'Heard: "{input_text}"')
        # Here you would typically get the person_id of the speaker
        # For now, we'll assume it's the last person we greeted or a default
        person_id = "person_1" 

        # Save the user's speech to memory
        self.save_line(person_id, "person", input_text)

        # Get conversation history and call the LLM stub for a response
        # In a real scenario, you'd get the history before calling the LLM
        response_text = self.call_llm_api([], input_text)
        
        # Speak the response
        self.publish_speech(response_text)
        self.save_line(person_id, "robot", response_text)

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
        """
        This is a functional, rule-based replacement for a real LLM API call.
        It provides simple, interactive responses based on keywords.
        """
        new_line = new_line.lower()
        if "hello" in new_line or "hi" in new_line:
            return "Hello there! What can I do for you?"
        elif "your name" in new_line:
            return "I am a friendly AI pet robot."
        elif "how are you" in new_line:
            return "I am running on all cylinders! Thanks for asking."
        elif "follow me" in new_line:
            # This is an example of how you could trigger other robot actions.
            # In a real implementation, you'd publish to a topic here.
            return "I can't follow you yet, but that feature is coming soon!"
        elif "what can you do" in new_line:
            return "I can recognize faces, remember our conversations, and navigate around. Ask me to do something!"
        else:
            responses = [
                "That's very interesting.",
                "Tell me more about that.",
                "I see. What else is on your mind?",
                "Fascinating. Please continue."
            ]
            # Return a different response each time
            return responses[datetime.datetime.now().second % len(responses)]


def main(args=None):
    rclpy.init(args=args)
    node = InteractionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
