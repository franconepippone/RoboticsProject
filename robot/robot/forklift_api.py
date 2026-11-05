from rclpy.node import Node
from std_msgs.msg import Float64


class ForkliftAPI:
    """Nicer api to the underlying forklift topic. Create an instance and pass a host node,
    then use 'lift' and 'lower' methods to interact with the forklift. 
    
    Forklift robot subscriber must be active. No warnings will be raised if it isn't.
    """
    def __init__(self, host_node: Node):
        self.host_node = host_node

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
        self.host_node.get_logger().debug('Forklift grab command: 0.5')

    def lower(self):
        """Lowers the forklift. Grabbed packages will be deposited."""
        msg = Float64()
        msg.data = 0.0
        self.publisher.publish(msg)
        self.host_node.get_logger().debug('Forklift release command: 0.0')
