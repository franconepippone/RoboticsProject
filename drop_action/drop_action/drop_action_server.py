import time
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_interfaces.action import DropPackage
from robot.forklift_api import ForkliftAPI

class DropActionServer(Node):

    def __init__(self, linear_x=0.2, duration=5.0, package_num=None, delivery_zone_id=None):
        super().__init__('drop_action_server')

        self.package_num = package_num
        self.delivery_zone_id = delivery_zone_id

        # Attributes for Undocking maneuver
        self.linear_x = linear_x
        self.duration = duration

        # Initialize Action Server
        self._action_server = ActionServer(
            self,
            DropPackage,
            'drop_package',  # Topic's name to call in the client
            self.execute_callback
        )

        # Initialize Forklift API -> To move robot
        self.forklift_api = ForkliftAPI(self)

        # Initialize Publisher on /cmd_vel topic -> To move robot
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        self.get_logger().info('Drop Action Server waiting for requests...')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received request to drop package. Starting operation...')
        self.get_logger().info(f"Package is number {self.package_num} in delivery zone {self.delivery_zone_id}.")

        # Initialize Feedback Message
        feedback_msg = DropPackage.Feedback()

        # Rotate 180 degrees - Robot arrives backwards
        feedback_msg.status = 'Aligning with delivery pose...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)

        angular_z = 0.5
        msg = Twist()
        msg.angular.z = angular_z
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=math.pi / angular_z)
        while (self.get_clock().now() - start_time) < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

        # Move forward to reach delivery spot - 1m away
        feedback_msg.status = 'Moving to delivery position...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)

        msg = Twist()
        msg.linear.x = -self.linear_x # Carrying is true => drive_mode node inverts
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=self.duration)
        while (self.get_clock().now() - start_time) < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())
        
        # Lower Forklift
        feedback_msg.status = 'Lowering forklift...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)
        self.forklift_api.lower()

        # Undock
        feedback_msg.status = 'Undocking...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)

        msg = Twist()
        msg.linear.x = -self.linear_x
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=self.duration)
        while (self.get_clock().now() - start_time) < duration:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

        # Conclude Goal
        goal_handle.succeed()

        # Return Result Message to Client
        result = DropPackage.Result()
        result.success = True
        result.message = 'Package dropped correctly!'
        self.get_logger().info(result.message)

        return result

def main(args=None):
    rclpy.init(args=args)
    drop_action_node = DropActionServer()
    
    try:
        rclpy.spin(drop_action_node)
    except KeyboardInterrupt:
        pass
    finally:
        drop_action_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()