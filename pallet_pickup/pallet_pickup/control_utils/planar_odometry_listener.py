import math
from typing import cast

from rclpy.node import Node
from nav_msgs.msg import Odometry
import threading

class PlanarOdometryListener:
    """Tracks planar position and unwrapped yaw from odometry."""

    def __init__(self, node: Node, topic='/odom'):
        self._lock = threading.Lock()

        self._x: float = 0.0
        self._y: float = 0.0
        self._yaw: float = 0.0
        self._unwrapped_yaw: float = 0.0

        self._subscription = node.create_subscription(
            Odometry,
            topic,
            self._callback,
            10,
        )

    def _callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        with self._lock:
            if self._yaw is None:
                self._unwrapped_yaw = yaw
            else:
                delta = math.atan2(
                    math.sin(yaw - self._yaw),
                    math.cos(yaw - self._yaw),
                )
                self._unwrapped_yaw += delta

            assert isinstance(p.x, float) and isinstance(p.y, float)

            self._x = p.x
            self._y = p.y
            self._yaw = yaw

    def get_pose(self) -> tuple[float, float, float]:
        with self._lock:
            if self._yaw is None:
                raise RuntimeError("No odometry received yet")

            return self._x, self._y, self._yaw

    def get_yaw(self) -> float:
        with self._lock:
            if self._yaw is None:
                raise RuntimeError("No odometry received yet")

            return self._yaw

    def get_unwrapped_yaw(self) -> float:
        with self._lock:
            if self._unwrapped_yaw is None:
                raise RuntimeError("No odometry received yet")

            return self._unwrapped_yaw


