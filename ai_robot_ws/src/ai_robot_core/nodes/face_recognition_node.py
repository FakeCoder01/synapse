import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ai_robot_msgs.msg import Person, PersonArray
from ai_robot_msgs.srv import LookupPerson, RegisterPerson
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        self.bridge = CvBridge()
        self.haar_cascade = cv2.CascadeClassifier(os.path.join(get_package_share_directory('ai_robot_core'), 'data', 'haarcascade_frontalface_default.xml'))
        
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.annotated_image_publisher = self.create_publisher(Image, '/annotated_image', 10)
        self.detected_people_publisher = self.create_publisher(PersonArray, '/detected_people', 10)
        
        self.lookup_person_client = self.create_client(LookupPerson, '/lookup_person')
        self.register_person_client = self.create_client(RegisterPerson, '/register_person')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces_rect = self.haar_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

        person_array_msg = PersonArray()
        person_array_msg.header = msg.header
        
        for (x, y, w, h) in faces_rect:
            face_encoding = np.random.rand(128).astype(np.float32) # Fake encoding

            lookup_req = LookupPerson.Request()
            lookup_req.face_encoding = face_encoding.tolist()
            
            future = self.lookup_person_client.call_async(lookup_req)
            rclpy.spin_until_future_complete(self, future)
            
            person = future.result().person_data

            if not person.is_known:
                register_req = RegisterPerson.Request()
                register_req.name_hint = "Unknown"
                register_req.face_encoding = face_encoding.tolist()
                
                future = self.register_person_client.call_async(register_req)
                rclpy.spin_until_future_complete(self, future)
                person.person_id = future.result().person_id
                person.name = "Unknown"
                person.is_known = True


            person.bounding_box = [int(x), int(y), int(x+w), int(y+h)]
            person_array_msg.people.append(person)

            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(cv_image, person.name, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)

        self.detected_people_publisher.publish(person_array_msg)
        
        try:
            annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.annotated_image_publisher.publish(annotated_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert annotated image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
