import math

from geometry_msgs.msg import Twist, Vector3

from .planar_odometry_listener import PlanarOdometryListener
from .twist_controller import TwistController



class OdometricPlanarController:
    """
    Allows for simple closed-loop planar navigation of a robot taking Twist commands and 
    publishing odometry messages.

    """

    def __init__(
        self,
        node,
        cmd_topic='/cmd_vel',
        odom_topic='/odom',
        frequency=20.0,

        drift_linear_coeff=1.0,
        drift_rot_coeff=1.0,

        linear_kp=1.0,
        linear_min=0.02,
        linear_max=0.15,
        linear_tolerance=0.001,

        angular_kp=1.0,
        angular_min=0.02,
        angular_max=0.4,
        angular_tolerance=0.001,
    ):
        self.node = node

        self.vel_control = TwistController(
            node=node,
            topic=cmd_topic,
            frequency=frequency,
        )

        self.robot_odom = PlanarOdometryListener(
            node,
            topic=odom_topic,
        )

        self.drift_linear_coeff = drift_linear_coeff
        self.drift_rot_coeff = drift_rot_coeff

        self.linear_kp = linear_kp
        self.linear_min = linear_min
        self.linear_max = linear_max
        self.linear_tolerance = linear_tolerance

        self.angular_kp = angular_kp
        self.angular_min = angular_min
        self.angular_max = angular_max
        self.angular_tolerance = angular_tolerance

    @staticmethod
    def _p_control(error: float, kp: float, min_input: float, max_input: float):
        if abs(error) == 0:
            return 0.0

        output = kp * error             # proportional control law
        output = math.copysign(             # apply min clamping
            max(min_input, abs(output)),
            output,
        )

        # apply max clamping
        return max(-max_input, min(output, max_input))

    async def forward_by(self, meters: float):
        """Move by meters in the current 'forward' direction
        """
        x0, y0, yaw0 = self.robot_odom.get_pose()

        target = meters * self.drift_linear_coeff

        def progress():
            # linear motion along the initial yaw direction since the start
            x, y, _ = self.robot_odom.get_pose()

            dx = x - x0
            dy = y - y0

            return dx * math.cos(yaw0) + dy * math.sin(yaw0)

        def command():
            error = target - progress()

            velocity = self._p_control(
                error,
                self.linear_kp,
                self.linear_min,
                self.linear_max,
            )

            return Twist(
                linear=Vector3(x=velocity),
            )

        def stop_condition():
            return abs(target - progress()) < self.linear_tolerance

        await self.vel_control.loop(command, stop_condition)

    async def rotate_by(self, radians: float):
        """Rotate by 'radians' on the spot."""

        start_angle = self.robot_odom.get_unwrapped_yaw()
        target_angle = start_angle + radians * self.drift_rot_coeff

        def command():
            error = target_angle - self.robot_odom.get_unwrapped_yaw()

            velocity = self._p_control(
                error,
                self.angular_kp,
                self.angular_min,
                self.angular_max,
            )

            return Twist(
                angular=Vector3(z=velocity),
            )

        def stop_condition():
            return abs(
                target_angle - self.robot_odom.get_unwrapped_yaw()
            ) < self.angular_tolerance

        await self.vel_control.loop(command, stop_condition)