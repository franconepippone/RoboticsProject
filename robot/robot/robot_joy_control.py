import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from robot.forklift_api import ForkliftAPI


class JoyControlNode(Node):
    """
    Node to read joypad inputs from /joy and translate them into /cmd_vel and forklift
    commands for the robot
    """

    def __init__(self):
        super().__init__('joy_control_node')
        self.declare_parameter('deadzone', value=0.1)

        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_joy',
            10,
        )

        self.forklift = ForkliftAPI(self)

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
        )

        # Typical PlayStation controller mapping.
        self.linear_axis = 1   # Left stick vertical
        self.angular_axis = 0  # Left stick horizontal
        self.r1_button = 5     # R1 / RB

        self.linear_speed = 1.0
        self.angular_speed = 1.0

        self.forklift_is_raised = False
        self.previous_buttons = []

        self.get_logger().info(
            'Joystick control ready: '
            'left stick = drive, R1 = toggle forklift'
        )

    def joy_callback(self, msg: Joy):
        # Drive using the left joystick.
        cmd = Twist()

        if len(msg.axes) > max(self.linear_axis, self.angular_axis):
            cmd.linear.x = self.linear_speed * msg.axes[self.linear_axis]
            cmd.angular.z = self.angular_speed * msg.axes[self.angular_axis]


        # only publish if input is above deadzone
        if abs(cmd.linear.x) > self.deadzone or abs(cmd.angular.z) > self.deadzone:
            self.cmd_vel_publisher.publish(cmd)

        # Initialize button history on the first message.
        if not self.previous_buttons:
            self.previous_buttons = list(msg.buttons)
            return

        # Toggle forklift state only when R1 transitions from
        # released to pressed.
        if self.r1_button < len(msg.buttons):
            pressed = msg.buttons[self.r1_button]
            was_pressed = (
                self.r1_button < len(self.previous_buttons)
                and self.previous_buttons[self.r1_button]
            )

            if pressed and not was_pressed:
                if self.forklift_is_raised:
                    self.forklift.lower()
                    self.forklift_is_raised = False
                    self.get_logger().info('Forklift lowered')
                else:
                    self.forklift.lift()
                    self.forklift_is_raised = True
                    self.get_logger().info('Forklift raised')

        self.previous_buttons = list(msg.buttons)


def main(args=None):
    rclpy.init(args=args)
    node = JoyControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()