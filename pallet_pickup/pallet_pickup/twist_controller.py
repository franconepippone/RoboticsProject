import rclpy
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Twist
from typing import Callable, Optional


class TwistController:
    """
    Allows periodic publication of a custom twist message on a topic until a
    stop condition is met; at that point, a final zero Twist message is published
    to clear out commands. 
    
    Stop condition and twist are re-evaluated at each publication,
    and are provided externally.

    Additionally, the 'loop' method is awaitable by a ros executor. 
    """
    def __init__(
        self,
        node: Node,
        topic: str = '/cmd_vel',
        frequency: float = 50.0,
    ):
        if frequency <= 0.0:
            raise ValueError("frequency must be positive")

        self._node = node
        self._frequency = frequency
        self._period = 1.0 / frequency

        self._publisher = node.create_publisher(Twist, topic, 10)
        self._timer = None
        self._future: Optional[Future] = None

        self._command_fn: Callable[[], Twist] = lambda: Twist()     # default zero twist
        self._stop_condition: Callable[[], bool] = lambda: False    # stops immediately

    def cancel(self):
        """Stops the controller, if running."""
        if self._future is not None and not self._future.done():
            self._future.set_result(None)

    async def loop(
        self,
        command_fn: Callable[[], Twist],
        stop_condition: Callable[[], bool],
    ) -> None:
        """
        Publish Twist provided by command_fn() until stop_condition() returns True.
        Awaitable by a ros executor, yields when stop condition is met.
        """
        if self._future is not None and not self._future.done():
            raise RuntimeError("Controller is already running")

        self._command_fn = command_fn
        self._stop_condition = stop_condition
        self._future = Future()

        self._timer = self._node.create_timer(
            self._period,
            self._control_loop,
        )

        await self._future

    def _control_loop(self):
        try:
            if self._stop_condition():
                self._stop()
                return

            command = self._command_fn()
            self._publisher.publish(command)

        except Exception as exc:
            self._stop(exc)

    def _stop(self, exception=None):
        if self._timer is not None:
            self._node.destroy_timer(self._timer)
            self._timer = None

        self._publisher.publish(Twist())

        if self._future is not None and not self._future.done():
            if exception is not None:
                self._future.set_exception(exception)
            else:
                self._future.set_result(None)