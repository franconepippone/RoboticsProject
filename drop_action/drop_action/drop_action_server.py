import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.task import Future
from custom_interfaces.action import DropPackage
from robot.forklift_api import ForkliftAPI

class DropActionServer(Node):

    def __init__(self, linear_x=0.2, duration=5.0):
        super().__init__('drop_action_server')

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

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received request to drop package. Starting operation...')

        # Initialize Feedback Message
        feedback_msg = DropPackage.Feedback()

        # Rotate 180 degrees - Robot arrives backwards
        feedback_msg.status = 'Aligning with delivery pose...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)

        angular_z = 0.5
        msg = Twist()
        msg.angular.z = angular_z
        await self._publish_for(msg, math.pi / 0.5)

        # Move forward to reach delivery spot - 1m away
        feedback_msg.status = 'Moving to delivery position...'
        self.get_logger().info(feedback_msg.status)
        goal_handle.publish_feedback(feedback_msg)

        msg = Twist()
        msg.linear.x = -self.linear_x # Carrying is true => drive_mode node inverts
        await self._publish_for(msg, self.duration)
        
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
        await self._publish_for(msg, self.duration)
        
        # Conclude Goal
        goal_handle.succeed()

        # Return Result Message to Client
        result = DropPackage.Result()
        result.success = True
        result.message = 'Package dropped correctly!'
        self.get_logger().info(result.message)

        return result

    async def _publish_for(self, msg, seconds, period=0.05):
        """Publish msg for `seconds` simulation time, then stop."""
        done = Future()
        start = self.get_clock().now()
        limit = Duration(seconds=seconds)

        def tick():
            if self.get_clock().now() - start >= limit:
                if not done.done():
                    done.set_result(None)
                return
            self.cmd_vel_pub.publish(msg)

        timer = self.create_timer(period, tick)
        try:
           await done     
        finally:
            self.destroy_timer(timer)
            self.cmd_vel_pub.publish(Twist())

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