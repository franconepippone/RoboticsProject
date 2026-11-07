#!/usr/bin/env python3
import rclpy
import tf2_ros
import py_trees
import py_trees_ros.trees
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from autonomy.helpers import make_pose
from autonomy.bt_generic_blocks import SETUP_WAIT_SEC
from autonomy.behavior_tree import create_root

Status = py_trees.common.Status


class AutonomyNode(Node):
    """Runs the given BT `root`: bootstrap (TF, initial pose, Nav2), setup and ticking."""

    def __init__(self, root):
        super().__init__("autonomy_node")

        for key in ("x", "y", "yaw"):
            self.declare_parameter(f"initial_pose.{key}", 0.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
        self.finished = []

    # ============= Bootstrap ============== #
    # - Wait for TF                          #
    # - Set initial pose (AMCL)              #
    # - Wait for Nav2                        #
    # ====================================== #
    def bootstrap(self):
        """Blocking: waits until the rest of the system is ready (uses spin_once)."""

        log = self.get_logger()
        log.info("=== BT SUPERVISOR: Bootstrap ===")
        log.info("Waiting for odometry TF (odom -> base_link)...")
        while rclpy.ok() and not self.tf_buffer.can_transform("odom", "base_link", Time()):
            rclpy.spin_once(self, timeout_sec=0.5)

        # Publish initial pose (sim time) until AMCL provides map -> odom
        initial_pose = make_pose(*(self.get_parameter(f"initial_pose.{k}").value for k in ("x", "y", "yaw")))
        log.info("Setting initial pose, waiting for map -> odom (AMCL)...")
        while rclpy.ok() and not self.tf_buffer.can_transform("map", "odom", Time()):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose = initial_pose.pose
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=1.0)

        # /go_to needs Nav2 active: wait for its action server
        log.info("Waiting for Nav2 navigate_to_pose server...")
        while rclpy.ok() and not self.nav_client.wait_for_server(timeout_sec=1.0):
            pass

    # =========== Behavior Tree ========== #
    def start(self) -> bool:
        """Setup the tree and start ticking. Returns False if setup fails."""

        log = self.get_logger()
        try: self.tree.setup(node=self, timeout=SETUP_WAIT_SEC)

        except Exception as e:
            log.error(f"Behavior tree setup failed due to: {e}")
            return False

        self.tree.add_post_tick_handler(
            lambda t: self.finished.append(t.root.status)
            if t.root.status in (Status.SUCCESS, Status.FAILURE) else None
        )
        log.info("=== SYSTEM FULLY INITIALIZED: Starting Mission ===")
        self.tree.tick_tock(period_ms=50.0)    # 20 Hz
        return True


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode(create_root())

    try:
        node.bootstrap()
        if node.start():
            while rclpy.ok() and not node.finished:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.tree.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()