#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from custom_interfaces.srv import GetTagPosition, SetForkHeight, ToggleAttachment
from irobot_create_msgs.action import Undock
import tf2_ros

class SupervisorNode(Node):
    def __init__(self):
        super().__init__(
            'supervisor_node',
            automatically_declare_parameters_from_overrides=True
        )

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.latest_detected_tag_id = None

        self.tag_sub = self.create_subscription(
            PointStamped,
            '/apriltag_detection/tag_detection',
            self.tag_detection_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tag_client = self.create_client(GetTagPosition, '/apriltag_detection/get_tag_position')
        self.fork_client = self.create_client(SetForkHeight, '/set_fork_height')
        self.attach_client = self.create_client(ToggleAttachment, '/toggle_attachment')
        self.undock_client = ActionClient(self, Undock, '/undock')

    def wait_for_services(self):
        self.get_logger().info("Waiting hardware and vision services...")
        self.tag_client.wait_for_service()
        self.fork_client.wait_for_service()
        self.attach_client.wait_for_service()
        self.get_logger().info("All services are ready.")

    def set_fork_height(self, height: float):
        req = SetForkHeight.Request()
        req.height = height
        future = self.fork_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def toggle_attachment(self, attach:bool):
        req = ToggleAttachment.Request()
        req.attach = attach
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def get_tag_position(self, tag_id: int):
        req = GetTagPosition.Request()
        req.tag_id = tag_id
        future = self.tag_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def manual_move(self, linear_x: float, duration_sec: float):
        twist = Twist()
        twist.linear.x = linear_x
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=duration_sec)
        while (self.get_clock().now() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_vel_pub.publish(Twist())

    def tag_detection_callback(self, msg: PointStamped):
        """Callback to save tag_id given from an AprilTag."""
        frame = msg.header.frame_id
        self.get_logger().info(f"Detected frame ID {frame}.")
        if ':=:' in frame:
            try:
                tag_id = int(frame.split(':=:')[1])
                self.latest_detected_tag_id = tag_id
                self.get_logger().info(f"Retrieved tag ID {tag_id} from detected frame.")
            except (ValueError, IndexError):
                pass

    def get_initial_pose(self, navigator):
        """Builds PoseStamped object for the initial position given as ROS parameters."""

        if self.has_parameter("initial_pose.x"):
            x = self.get_parameter('initial_pose.x').value
            y = self.get_parameter('initial_pose.y').value
            yaw = self.get_parameter('initial_pose.yaw').value
            self.get_logger().info(f"Initializing position with values x: {x}, y: {y} and yaw: {yaw}.")
        else:
            self.get_logger().warn("Initial pose not given in YAML. Using default (0.0, 0.0, 0.0)")
            x, y, yaw = 0.0, 0.0, 0.0

        # Converts from yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # Build object
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = float(x)
        initial_pose.pose.position.y = float(y)
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw 
        
        return initial_pose

    def get_storage_location(self):
        """Builds PoseStamped object for the storage location given as ROS parameters."""
        
        if self.has_parameter("storage_location.x"):
            x = self.get_parameter('storage_location.x').value
            y = self.get_parameter('storage_location.y').value
            yaw = self.get_parameter('storage_location.yaw').value
            return {'x': x, 'y': y, 'yaw': yaw}
        else:
            self.get_logger().warn("Storage location not given in YAML. Using default (5.0, 0.0, 0.0)")
            return {'x': 5.0, 'y': 0.0, 'yaw': 0.0}

    def detect_tag_at_storage(self, timeout_sec:float = 10.0):
        "When in storage zone, waits for the AprilTag to be read to get the tag_id"

        self.get_logger().info("Scanning package's AprilTag for info...")
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=timeout_sec)

        self.latest_detected_tag_id = None
        while (self.get_clock().now() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.latest_detected_tag_id is not None:
                detected_id = self.latest_detected_tag_id
                self.get_logger().info(f"AprilTag scenned: Tag ID {detected_id}")
                return detected_id

        self.get_logger().error("Timeout: No AprilTag detected at storage location.")
        return -1

    def get_delivery_location(self, tag_id:int):
        """Reads correct delivery position for a given tag_id from YAML config file"""
        prefix = f"delivery_locations.tag_{tag_id}"
        if self.has_parameter(f"{prefix}.x"):
            x = self.get_parameter(f"{prefix}.x").value
            y = self.get_parameter(f"{prefix}.y").value
            yaw = self.get_parameter(f"{prefix}.yaw").value
            return {'x': x, 'y': y, 'yaw': yaw}
        else:
            self.get_logger().error(f"No delivery position found for tag_id {tag_id}.")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    navigator = BasicNavigator()

    # Wait until everything is in place to make the robot work
    node.get_logger().info("=== SUPERVISOR STARTED: Waiting for system readiness and all services to be available ===")

    node.get_logger().info("[1/4] Waiting for hardware, vision and undock services...")
    node.wait_for_services()
    while not node.undock_client.wait_for_server(timeout_sec=2.0):
        node.get_logger().info("Waiting for '/undock' action server...")
    node.get_logger().info("All ROS 2 services and action servers are ONLINE.")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    node.get_logger().info("Waiting for odometry TF (odom -> base_link) to become available...")
    while rclpy.ok():
        try:
            if tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()):
                break
        except Exception:
            pass
        rclpy.spin_once(node, timeout_sec=0.5)

    # Set initial pose
    node.get_logger().info("[2/4] Publishing initial pose to AMCL...")
    initial_pose = node.get_initial_pose(navigator)
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)

    node.get_logger().info("[3/4] Waiting for Nav2 stack (AMCL, Planner, Controller) to become ACTIVE...")
    navigator.waitUntilNav2Active()
    node.get_logger().info("Nav2 stack is fully ACTIVE.")

    # Execute undocking procedure
    # Provide manual fallback in case undock action fails
    node.get_logger().info("Starting undocking maneuver...")
    goal_msg = Undock.Goal()
    future = node.undock_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    undock_successful = False
    if future.done():
        goal_handle = future.result()
        if not goal_handle.accepted:
                node.get_logger().error("Undock request rejected!")
        elif goal_handle and goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            # Wait for the action to complete with a 10-second timeout
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
            if result_future.done():
                undock_successful = True
                node.get_logger().info("Undocking action completed successfully via Nav2!")

    # Fallback: If the /undock action failed or timed out, reverse manually
    if not undock_successful:
        node.get_logger().warn("Undock action server timed out or failed. Executing manual reverse fallback...")
        node.manual_move(linear_x=-0.15, duration_sec=3.0)
        node.get_logger().info("Manual undocking fallback completed.")

    # Verify map -> base_link transform is available
    node.get_logger().info("[4/4] Verifying map->base_link TF transform availability...")
    tf_ready = False
    while not tf_ready:
        try:
            if tf_buffer.can_transform('map', 'base_link', Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                tf_ready = True
                node.get_logger().info("TF transform 'map' -> 'base_link' is VALID.")
            else:
                node.get_logger().info("Waiting for AMCL to publish 'map' frame...")
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info("=== SYSTEM FULLY INITIALIZED: Starting Mission ===")

    # Build storage location PoseStamped object
    storage_location = node.get_storage_location()
    storage_pose = PoseStamped()
    storage_pose.header.frame_id = 'map'
    storage_pose.header.stamp = navigator.get_clock().now().to_msg() 
    storage_pose.pose.position.x = float(storage_location['x'])
    storage_pose.pose.position.y = float(storage_location['y']) 
    storage_pose.pose.orientation.z = math.sin(storage_location['yaw'] / 2.0)
    storage_pose.pose.orientation.w = math.cos(storage_location['yaw'] / 2.0)

    # Go to storage location
    node.get_logger().info("Moving towards storage location...")
    navigator.goToPose(storage_pose)
    while not navigator.isTaskComplete():
        rclpy.spin_once(node, timeout_sec=0.1)
        
    if navigator.getResult() != TaskResult.SUCCEEDED:
        node.get_logger().error("Impossible to reach storage location.")
        rclpy.shutdown()
        return
    node.get_logger().info("Storage location reached.")

    # Detect package position
    target_tag_id = node.detect_tag_at_storage(timeout_sec=10.0)
    if target_tag_id == -1:
        node.get_logger().error("Mission completed: no other package detected.")
        rclpy.shutdown()
        return
    
    tag_res = node.get_tag_position(target_tag_id)
    dest = node.get_delivery_location(target_tag_id)
    if not tag_res.found:
        node.get_logger().error(f"Error. Couldn't get package's coordinates for package with tag ID {target_tag_id}")
        rclpy.shutdown()
        return
    if not dest:
        node.get_logger().error(f"No delivery zone associated to tag ID {target_tag_id}.")
        rclpy.shutdown()
        return
    
    node.get_logger().info(f"Package's coordinates found: X={tag_res.position.point.x:.2f}, Y={tag_res.position.point.y:.2f}")

    # Take package
    node.get_logger().info("Lowering fork...")
    node.set_fork_height(0.0)

    node.get_logger().info("Precision docking towards package...")
    node.manual_move(linear_x=0.1, duration_sec=2.5)
    
    node.get_logger().info("Attach package...")
    node.toggle_attachment(attach=True)
    
    node.get_logger().info("Lifting fork...")
    node.set_fork_height(0.10)
    
    node.get_logger().info("Backing from storage location...")
    node.manual_move(linear_x=-0.1, duration_sec=2.0)

    # Build delivery zone PoseStamped object
    delivery_pose = PoseStamped()
    delivery_pose.header.frame_id = 'map'
    delivery_pose.header.stamp = navigator.get_clock().now().to_msg()
    delivery_pose.pose.position.x = float(dest['x'])
    delivery_pose.pose.position.y = float(dest['y'])
    delivery_pose.pose.orientation.z = math.sin(dest['yaw'] / 2.0)
    delivery_pose.pose.orientation.w = math.cos(dest['yaw'] / 2.0)

    # Go to delivery zone
    node.get_logger().info(f"Taking the package to delivery zone for tag ID {target_tag_id}...")
    navigator.goToPose(delivery_pose)
    
    while not navigator.isTaskComplete():
        rclpy.spin_once(node, timeout_sec=0.1)
        
    if navigator.getResult() == TaskResult.SUCCEEDED:
        node.get_logger().info("Delivery zone reached! Detaching package...")
        node.set_fork_height(0.0)
        node.toggle_attachment(attach=False)
        node.manual_move(linear_x=-0.1, duration_sec=2.0)
        node.set_fork_height(0.10)
        node.get_logger().info("--- Mission Completed ---")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()