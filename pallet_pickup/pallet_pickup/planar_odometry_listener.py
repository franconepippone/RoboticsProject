import math

from rclpy.node import Node
from nav_msgs.msg import Odometry


class PlanarOdometryListener:
    """Listens to an odometry topic, and exposes getter methods for robot state.
    Ment to be used for planar robots only.
    """
    def __init__(
        self,
        node: Node,
        topic: str = '/odom',
        qos_depth: int = 10,
    ):
        self._yaw = None

        self._subscription = node.create_subscription(
            Odometry,
            topic,
            self._odom_callback,
            qos_depth,
        )

    def _odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation

        self._yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def get_angle(self) -> float:
        """Get the yaw of the robot."""
        if self._yaw is None:
            raise RuntimeError("No odometry received yet")

        return self._yaw