import time
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class ForkliftAPI:
    """Nicer api to the underlying forklift topic. Create an instance and pass a host node,
    then use 'lift' and 'lower' methods to interact with the forklift. 
    
    Forklift robot subscriber must be active. No warnings will be raised if it isn't.
    """
    def __init__(self, host_node: Node):
        self.host_node = host_node
        self.carrying_client = self.host_node.create_client(SetBool, '/set_carrying')

        self.publisher = self.host_node.create_publisher(
            Float64,
            '/forklift/lift/position',
            10,
        )

    def lift(self):
        """Lifts the forklift. Packages on top will be grabbed."""
        msg = Float64()
        msg.data = 0.5
        self.publisher.publish(msg)
        self._set_carrying(True)
        time.sleep(0.2)
        self.host_node.get_logger().info('Forklift grab command: 0.5')

    def lower(self):
        """Lowers the forklift. Grabbed packages will be deposited."""
        msg = Float64()
        msg.data = 0.0
        self.publisher.publish(msg)
        self._set_carrying(False)
        time.sleep(0.2)
        self.host_node.get_logger().info('Forklift release command: 0.0')

    def _set_carrying(self, value: bool):
       req = SetBool.Request()
       req.data = value
       self.carrying_client.call_async(req)
