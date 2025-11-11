import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from ai_robot_msgs.msg import PersonArray

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')
        self.following_id = None
        self.desired_area = 60 * 80 # A target size for the bounding box

        self.detected_people_subscription = self.create_subscription(
            PersonArray,
            '/detected_people',
            self.detected_people_callback,
            10)
        
        self.start_follow_subscription = self.create_subscription(
            String,
            '/follow_person_start',
            self.start_follow_callback,
            10)
        
        self.stop_follow_subscription = self.create_subscription(
            Empty,
            '/follow_person_stop',
            self.stop_follow_callback,
            10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def start_follow_callback(self, msg):
        self.following_id = msg.data
        self.get_logger().info(f"Starting to follow person: {self.following_id}")

    def stop_follow_callback(self, msg):
        self.following_id = None
        self.get_logger().info("Stopping person following")
        # Stop the robot
        self.cmd_vel_publisher.publish(Twist())


    def detected_people_callback(self, msg):
        if self.following_id is None:
            return

        target_person = None
        for person in msg.people:
            if person.person_id == self.following_id:
                target_person = person
                break
        
        if target_person:
            twist = Twist()
            
            # P-controller for angular velocity
            image_center_x = 320 # for a 640x480 image
            person_center_x = target_person.bounding_box[0] + (target_person.bounding_box[2] - target_person.bounding_box[0]) / 2
            angular_error = image_center_x - person_center_x
            twist.angular.z = -0.002 * angular_error

            # P-controller for linear velocity
            person_area = (target_person.bounding_box[2] - target_person.bounding_box[0]) * (target_person.bounding_box[3] - target_person.bounding_box[1])
            linear_error = self.desired_area - person_area
            twist.linear.x = 0.00001 * linear_error

            # Clamp velocities
            twist.linear.x = max(min(twist.linear.x, 0.2), -0.2)
            twist.angular.z = max(min(twist.angular.z, 0.5), -0.5)

            self.cmd_vel_publisher.publish(twist)
        else:
            # Stop the robot if the person is lost
            self.cmd_vel_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
