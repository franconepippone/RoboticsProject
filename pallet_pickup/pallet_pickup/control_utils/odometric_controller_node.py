import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from custom_interfaces.srv import MotionRequest

from .odometric_planar_controller import OdometricPlanarController


class OdometricControllerNode(Node):
    """Helper node for debug usage of the odometric planar controller from terminal.
    
    Completely configurable via parameters. Exposes /forward_by and /rotate_by services.
    """

    def __init__(self):
        super().__init__('odometric_controller_node')

        # Topics and execution frequency
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom_nav')
        self.declare_parameter('frequency', 20.0)

        # Drift compensation
        self.declare_parameter('drift_linear_coeff', 1.0)
        self.declare_parameter('drift_rot_coeff', 1.0)

        # Linear controller
        self.declare_parameter('linear_kp', 1.0)
        self.declare_parameter('linear_min', 0.02)
        self.declare_parameter('linear_max', 0.15)
        self.declare_parameter('linear_tolerance', 0.001)

        # Angular controller
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('angular_min', 0.02)
        self.declare_parameter('angular_max', 0.4)
        self.declare_parameter('angular_tolerance', 0.001)

        # Service names
        self.declare_parameter('forward_service', '/forward_by')
        self.declare_parameter('rotate_service', '/rotate_by')

        getd = lambda name: self.get_parameter(name).get_parameter_value().double_value
        getstr = lambda name: self.get_parameter(name).get_parameter_value().string_value
        

        self.controller = OdometricPlanarController(
            node=self,
            cmd_topic=getstr('cmd_topic'),
            odom_topic=getstr('odom_topic'),
            frequency=getd('frequency'),

            drift_linear_coeff=getd('drift_linear_coeff'),
            drift_rot_coeff=getd('drift_rot_coeff'),

            linear_kp=getd('linear_kp'),
            linear_min=getd('linear_min'),
            linear_max=getd('linear_max'),
            linear_tolerance=getd('linear_tolerance'),

            angular_kp=getd('angular_kp'),
            angular_min=getd('angular_min'),
            angular_max=getd('angular_max'),
            angular_tolerance=getd('angular_tolerance'),
        )

        # Keep service callbacks separate from the controller's
        # default callback group, where its timer runs.
        self._service_group = MutuallyExclusiveCallbackGroup()

        self.forward_srv = self.create_service(
            MotionRequest,
            getstr('forward_service'),
            self._forward_callback,
            callback_group=self._service_group,
        )

        self.rotate_srv = self.create_service(
            MotionRequest,
            getstr('rotate_service'),
            self._rotate_callback,
            callback_group=self._service_group,
        )

        self.get_logger().info('Odometric Planar Controller node ready')

    async def _forward_callback(self, request, response):
        try:
            if not math.isfinite(request.amount):
                raise ValueError('Distance must be finite')

            self.get_logger().info(
                f'Received forward command: {request.amount} m'
            )

            await self.controller.forward_by(request.amount)

        except Exception as exc:
            self.get_logger().error(f'Forward motion failed: {exc}')

        return response

    async def _rotate_callback(self, request, response):
        try:
            if not math.isfinite(request.amount):
                raise ValueError('Angle must be finite')

            self.get_logger().info(
                f'Received rotation command: {request.amount} rad'
            )

            await self.controller.rotate_by(request.amount)

        except Exception as exc:
            self.get_logger().error(f'Rotation failed: {exc}')

        return response


def main(args=None):
    rclpy.init(args=args)

    node = OdometricControllerNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()