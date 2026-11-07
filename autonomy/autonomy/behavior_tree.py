#!/usr/bin/env python3
import rclpy
import tf2_ros
import py_trees
import py_trees_ros
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import BatteryState

from custom_interfaces.action import PickupPallet, GoToDeliveryZone, DropPackage 
from custom_interfaces.srv import SelectTag 

from helpers import make_pose, pose_from_params, nav_goal
from bt_generic_blocks import BBBehaviour, TopicToBlackboard, ActionCall, ServiceCall, SETUP_WAIT_SEC

Status = py_trees.common.Status
Access = py_trees.common.Access

BATTERY_LOW = 0.30      # Threshold to check when starting a delivery cycle
BATTERY_FULL = 1.00     # Threshold to consider the robot charged

# =============== #
# Condition Nodes #
# =============== #
class BatteryOK(BBBehaviour):
    """Reads BB to know battery level.
       Once started charging (because under `low` threshold) keep charging until battery overcomes `full` threshold."""

    def __init__(self, name="BatteryOK", low=BATTERY_LOW, full=BATTERY_FULL):
        super().__init__(name, reads=["battery_percentage"])
        self.low, self.full, self.charging = low, full, False

    def update(self):
        level = self.bb_get("battery_percentage")
        if level is None:
            self.feedback_message = "Waiting for /battery_state"
            return Status.RUNNING
        if self.charging:
            if level >= self.full:
                self.charging = False
                self.feedback_message = f"Charge completed ({level:.0%})"
                return Status.SUCCESS
            self.feedback_message = f"Charging ({level:.0%})"
            return Status.FAILURE
        if level >= self.low:
            self.feedback_message = f"Battery ok ({level:.0%})"
            return Status.SUCCESS
        self.charging = True
        self.feedback_message = f"Battery low ({level:.0%})"
        return Status.FAILURE

class IsUndocked(BBBehaviour):
    def __init__(self, name="IsUndocked"):
        super().__init__(name, reads=["is_docked"])

    def update(self):
        docked = self.bb_get("is_docked")
        if docked is None:
            self.feedback_message = "Waiting for /dock_status"
            return Status.RUNNING
        return Status.FAILURE if docked else Status.SUCCESS

class TFReady(py_trees.behaviour.Behaviour):
    """RUNNING until (target, source) TF are available."""

    def __init__(self, name, buffer, pairs):
        super().__init__(name)
        self.buffer, self.pairs = buffer, pairs

    def update(self):
        for target, source in self.pairs:
            if not self.buffer.can_transform(target, source, Time()):
                self.feedback_message = f"Waiting for {target} -> {source}"
                return Status.RUNNING
        self.feedback_message = "TF available and valid."
        return Status.SUCCESS

# ============ #
# Action Nodes #
# ============ #

class ManualMove(BBBehaviour):
    """Publish Twist on /cmd_vel for `duration_sec` seconds"""

    def __init__(self, name, linear_x, duration_sec):
        super().__init__(name)
        self.linear_x, self.duration = linear_x, Duration(seconds=duration_sec)
        self.pub, self._start = None, None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.pub = self.node.create_publisher(Twist, "/cmd_vel", 10)

    def initialise(self):
        self._start = self.node.get_clock().now()
        self.node.get_logger().info(f"[{self.name}] v={self.linear_x} m/s")

    def update(self):
        if (self.node.get_clock().now() - self._start) >= self.duration:
            self.pub.publish(Twist())
            return Status.SUCCESS
        twist = Twist()
        twist.linear.x = self.linear_x
        self.pub.publish(twist)
        return Status.RUNNING

    def terminate(self, new_status):
        # If interrupted stop the robot
        if new_status == Status.INVALID and self.pub is not None:
            self.pub.publish(Twist())  

class ReportEnd(BBBehaviour):
    """Once loop finishes reports mission ending reason or error."""

    def __init__(self, name="ReportEnd"):
        super().__init__(name, reads=["mission_end_reason"])

    def update(self):
        if self.bb_get("mission_end_reason") == "no_more_packages":
            self.node.get_logger().info("--- Mission Completed: no other package detected ---")
            return Status.SUCCESS
        self.node.get_logger().error("--- Mission aborted: some node of the cycle failed. ---")
        return Status.FAILURE

def navigate(name, make_goal, reads=()):
    return ActionCall(
        name, NavigateToPose, "navigate_to_pose", make_goal, reads=reads,
        format_feedback=lambda fb: f"{fb.feedback.distance_remaining:.2f} m remaining")

# ========== #
# Build Tree #
# ========== #
def create_root(tf_buffer, storage_pose, dock_pose):

    # Write perceptions from topics to blackboard
    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=False)
    topics2bb.add_children([
        TopicToBlackboard("Battery2BB", "/battery_state", BatteryState, ["battery_percentage"],
                          lambda m: {"battery_percentage": m.percentage}),
        TopicToBlackboard("Dock2BB", "/dock_status", DockStatus, ["is_docked"],
                          lambda m: {"is_docked": m.is_docked}),
    ])

    # Check Battery
    # If not ok go to dock station, dock and charge
    charge = py_trees.composites.Sequence(name="Charge", memory=True)
    charge.add_children([
        navigate("GoToDock", lambda _bb: nav_goal(dock_pose)),
        ActionCall("Dock", Dock, "/dock", lambda _bb: Dock.Goal()),
        py_trees.behaviours.Running(name="Charging"),   # until BatteryOK not return SUCCESS
    ])
    battery_manager = py_trees.composites.Selector(name="BatteryManager", memory=False)
    battery_manager.add_children([BatteryOK(), charge])

    # Undock
    undock = ActionCall("Undock", Undock, "/undock", lambda _bb: Undock.Goal())
    undock_manager = py_trees.composites.Selector(name="UndockManager", memory=False)
    undock_manager.add_children([IsUndocked(), undock])

    # Go to storage, scand tags to choose package and pick one
    pickup_package = py_trees.composites.Sequence(name="PickupPackage", memory=True)
    pickup_package.add_children([
        navigate("GoToStorage", lambda _bb: nav_goal(storage_pose)),
        ServiceCall("SelectTag", SelectTag, "/custom_services/srv/SelectTag", TODO),
        ActionCall("PickupPackage", PickupPallet, "/custom_services/action/PickupPallet", TODO),
        py_trees.behaviours.Running(name="Picking Package..."),
    ])

    # Go to delivery zone and drop package
    deliver_package = py_trees.composites.Sequence(name="DeliverPackage", memory=True)
    deliver_package.add_children([
        ActionCall("GoToDeliveryZone", GoToDeliveryZone, "/custom_services/action/GoToDeliveryZone", TODO),
        ActionCall("DropPackage", DropPackage, "/custom_services/action/DropPackage", TODO),
        py_trees.behaviours.Running(name="Delivering Package..."),
    ])

    # Complete Delivery Cycle
    cycle = py_trees.composites.Sequence(name="DeliveryCycle", memory=True)
    cycle.add_children([
        battery_manager,
        undock_manager,
        TFReady("MapTF", tf_buffer, [("map", "base_link")]),
        pickup_package,
        deliver_package
    ])

    # SuccessIsRunning: when cycle ends (SUCCESS) decorator returns RUNNING and cycle restarts
    # Once cycle returns FAILURE (no other packages, or some error occurred) exit the loop.
    loop = py_trees.decorators.SuccessIsRunning(name="Loop", child=cycle)
    mission = py_trees.composites.Selector(name="Mission", memory=True)
    mission.add_children([loop, ReportEnd()])

    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    root.add_children([topics2bb, mission])
    return root

def main(args=None):
    
    # ============= Bootstrap ============== #
    # - Wait for TF and Nav2 Services       #
    # - Build PoseStamped objs for fixed POI #
    # ====================================== #

    rclpy.init(args=args)
    node = rclpy.create_node("autonomy_node", automatically_declare_parameters_from_overrides=True)
    log = node.get_logger()

    log.info("=== BT SUPERVISOR: Bootstrap ===")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    log.info("Waiting for odometry TF (odom -> base_link)...")
    while rclpy.ok() and not tf_buffer.can_transform("odom", "base_link", Time()):
        rclpy.spin_once(node, timeout_sec=0.5)

    # Build POI PoseStamped objs
    initial_pose = pose_from_params(node, "initial_pose")
    if initial_pose is None:
        log.warn("Initial pose not given in YAML. Using default (0.0, 0.0, 0.0)")
        initial_pose = make_pose(0.0, 0.0, 0.0)

    storage_pose = pose_from_params(node, "storage_location")
    if storage_pose is None:
        log.warn("Storage location not given in YAML. Using default (5.0, 0.0, 0.0)")
        storage_pose = make_pose(5.0, 0.0, 0.0)

    dock_pose = pose_from_params(node, "dock_location")
    if dock_pose is None:
        log.warn("dock_location not given in YAML. Using initial_pose as dock location pose")
        dock_pose = initial_pose

    navigator = BasicNavigator()
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)
    log.info("Waiting for Nav2 stack to become ACTIVE...")
    navigator.waitUntilNav2Active()
    navigator.destroy_node()          # Nav2 only used via action nodes

    # =========== Behavior Tree ========== #
    tree = py_trees_ros.trees.BehaviourTree(
        root=create_root(tf_buffer, storage_pose, dock_pose),
        unicode_tree_debug=False,
    )
    try:
        tree.setup(node=node, timeout=SETUP_WAIT_SEC)
    except Exception as e:
        log.error(f"Setup dell'albero fallito: {e}")
        tree.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        return

    # =========== Mission ========== #
    finished = []
    tree.add_post_tick_handler(
        lambda t: finished.append(t.root.status) if t.root.status in (Status.SUCCESS, Status.FAILURE) else None
    )
    log.info("=== SYSTEM FULLY INITIALIZED: Starting Mission ===")
    tree.tick_tock(period_ms=50.0)    # 20 Hz

    try:
        while rclpy.ok() and not finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()