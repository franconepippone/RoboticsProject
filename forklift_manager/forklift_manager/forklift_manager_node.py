import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Empty
from custom_interfaces.srv import SetForkHeight, ToggleAttachment

class ForkliftManager(Node):
    def __init__(self):
        super().__init__('forklift_manager')
        
        # Publisher for the prismatic joint position
        self.lift_pub = self.create_publisher(
            Float64, 
            '/forklift_fork/lift_joint/cmd_pos', 
            10
        )
        
        # Publishers for attach/detach commands (Empty messages)
        self.attach_pub = self.create_publisher(
            Empty, 
            '/forklift_fork/package/attach', 
            10
        )
        self.detach_pub = self.create_publisher(
            Empty, 
            '/forklift_fork/package/detach', 
            10
        )
        
        # Services
        self.height_srv = self.create_service(
            SetForkHeight, 
            'set_fork_height', 
            self.height_callback
        )
        
        self.attach_srv = self.create_service(
            ToggleAttachment,
            'toggle_attachment',
            self.attachment_callback
        )
        
        self.get_logger().info("Forklift Manager fully initialized and ready.")

    def height_callback(self, request, response):
        if 0.0 <= request.height <= 0.15:
            msg = Float64()
            msg.data = request.height
            self.lift_pub.publish(msg)
            
            response.success = True
            response.message = f"Command sent: fork moving to {request.height} m"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"Error: {request.height} m is out of bounds (0.0 - 0.15 m)"
            self.get_logger().warn(response.message)
            
        return response

    def attachment_callback(self, request, response):
        empty_msg = Empty()
        if request.attach:
            self.attach_pub.publish(empty_msg)
            response.success = True
            response.message = "Attach command sent to Gazebo."
            self.get_logger().info(response.message)
        else:
            self.detach_pub.publish(empty_msg)
            response.success = True
            response.message = "Detach command sent to Gazebo."
            self.get_logger().info(response.message)
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ForkliftManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()