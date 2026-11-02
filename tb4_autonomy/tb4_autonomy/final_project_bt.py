#!/usr/bin/env python3
import math
import time

import py_trees
import py_trees_ros
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from custom_interfaces.srv import GetTagPosition, SetForkHeight, ToggleAttachment
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState

Status = py_trees.common.Status
Access = py_trees.common.Access

BATTERY_LOW = 0.30      # sotto questa soglia, a inizio ciclo, si va a caricare
BATTERY_FULL = 0.80     # si resta in carica fino a questa soglia
SETUP_WAIT_SEC = 30.0   # attesa massima di ogni servizio / action server in setup


# ==========================================================================
# Pose Helpers
# ==========================================================================
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def pose_from_params(node, prefix: str):
    """PoseStamped da <prefix>.x/.y/.yaw nei parametri ROS, oppure None."""
    if not node.has_parameter(f"{prefix}.x"):
        return None
    return make_pose(
        node.get_parameter(f"{prefix}.x").value,
        node.get_parameter(f"{prefix}.y").value,
        node.get_parameter(f"{prefix}.yaw").value,
    )


def nav_goal(pose: PoseStamped) -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    goal.pose = pose
    return goal


# ==========================================================================
# Generic Blocks
# ==========================================================================
class BBBehaviour(py_trees.behaviour.Behaviour):
    """Base client for blackboard for passed keys in `reads` and `writes` with ROS node passed in setup()."""

    def __init__(self, name, reads=(), writes=()):
        super().__init__(name)
        self.node = None
        self.bb = self.attach_blackboard_client(name=name)
        for key in reads:
            self.bb.register_key(key=key, access=Access.READ)
        for key in writes:
            self.bb.register_key(key=key, access=Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def bb_get(self, key, default=None):
        try:
            return self.bb.get(key)
        except KeyError:
            return default


class TopicToBlackboard(BBBehaviour):
    """Copy topic's data on a blackboard. Always return SUCCESS.
    `transform(msg)` returns dict {key: value} or None to ignore message.
    If `count_key` given, count valid messages received.
    """

    def __init__(self, name, topic, msg_type, keys, transform, count_key=None):
        writes = list(keys) + ([count_key] if count_key else [])
        super().__init__(name, writes=writes)
        self.topic, self.msg_type = topic, msg_type
        self.transform, self.count_key = transform, count_key
        self._pending, self._count = {}, 0

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.node.create_subscription(self.msg_type, self.topic, self._on_msg, qos_profile_sensor_data)

    def _on_msg(self, msg):
        data = self.transform(msg)
        if data is not None:
            self._pending.update(data)
            self._count += 1

    def update(self):
        if self._pending:
            for key, value in self._pending.items():
                self.bb.set(key, value)
            if self.count_key:
                self.bb.set(self.count_key, self._count)
            self._pending = {}
        return Status.SUCCESS


class ActionCall(BBBehaviour):
    """Send a Goal to an Action server and remain RUNNING until it finishes."""

    def __init__(self, name, action_type, action_name, make_goal, reads=(), format_feedback=None):
        super().__init__(name, reads=reads)
        self.action_type, self.action_name = action_type, action_name
        self.make_goal, self.format_feedback = make_goal, format_feedback
        self.client = None
        self._goal_future = self._result_future = self._goal_handle = None
        self._active, self._error = False, None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.client = ActionClient(self.node, self.action_type, self.action_name)
        if not self.client.wait_for_server(timeout_sec=SETUP_WAIT_SEC):
            raise RuntimeError(f"[{self.name}] action server '{self.action_name}' not available")

    def initialise(self):
        self._goal_future = self._result_future = self._goal_handle = None
        self._active, self._error = False, None
        try:
            goal = self.make_goal(self.bb)
        except KeyError as e:
            self._error = f"Missing data on blackboard: {e}"
            return
        self.node.get_logger().info(f"[{self.name}] sending goal to '{self.action_name}'")
        self._goal_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        self._active = True

    def _on_feedback(self, fb):
        if self.format_feedback: self.feedback_message = self.format_feedback(fb)

    def update(self):
        if self._error:
            self.feedback_message = self._error
            return Status.FAILURE
        
        if self._result_future is None:
            if not self._goal_future.done():
                return Status.RUNNING
            
            handle = self._goal_future.result()
            if not handle.accepted:
                self._active = False
                self.feedback_message = "Goal Refused"
                return Status.FAILURE
            
            self._goal_handle = handle
            self._result_future = handle.get_result_async()
            return Status.RUNNING
        
        if not self._result_future.done():
            return Status.RUNNING
        
        self._active = False
        code = self._result_future.result().status
        if code == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "Completed"
            return Status.SUCCESS
        self.feedback_message = f"Ended with status {code}"
        return Status.FAILURE

    def terminate(self, new_status):
        if not self._active: return

        self._active = False
        if self._goal_handle is not None: self._goal_handle.cancel_goal_async()
        elif self._goal_future is not None: self._goal_future.add_done_callback(self._cancel_when_accepted)

    @staticmethod
    def _cancel_when_accepted(future):
        handle = future.result()
        if handle.accepted: handle.cancel_goal_async()


class ServiceCall(BBBehaviour):
    """Call a service (non-blocking).
    `on_response(bb, res) -> bool` callback determines how response is handled."""

    def __init__(self, name, srv_type, srv_name, make_request, on_response=None,
                 reads=(), writes=(), timeout=10.0):
        super().__init__(name, reads=reads, writes=writes)
        self.srv_type, self.srv_name = srv_type, srv_name
        self.make_request, self.on_response, self.timeout = make_request, on_response, timeout
        self.client = None
        self._future, self._deadline, self._error = None, 0.0, None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.client = self.node.create_client(self.srv_type, self.srv_name)
        if not self.client.wait_for_service(timeout_sec=SETUP_WAIT_SEC):
            raise RuntimeError(f"[{self.name}] service '{self.srv_name}' not available")

    def initialise(self):
        self._future, self._error = None, None
        self._deadline = time.monotonic() + self.timeout
        try:
            request = self.make_request(self.bb)
        except KeyError as e:
            self._error = f"Missing data on the blackboard: {e}"
            return
        self.node.get_logger().info(f"[{self.name}] call '{self.srv_name}'")
        self._future = self.client.call_async(request)

    def update(self):
        if self._error:
            self.feedback_message = self._error
            return Status.FAILURE
        if not self._future.done():
            if time.monotonic() > self._deadline:
                self.feedback_message = "timeout"
                return Status.FAILURE
            return Status.RUNNING
        ok = self.on_response(self.bb, self._future.result()) if self.on_response else True
        self.feedback_message = "ok" if ok else "not ok"
        return Status.SUCCESS if ok else Status.FAILURE


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

# ==========================================================================
# Condition Nodes
# ==========================================================================
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
    """RUNNING until all (target, source) TF are available."""

    def __init__(self, name, buffer, pairs):
        super().__init__(name)
        self.buffer, self.pairs = buffer, pairs

    def update(self):
        for target, source in self.pairs:
            if not self.buffer.can_transform(target, source, Time()):
                self.feedback_message = f"Waiting for {target} -> {source}"
                return Status.RUNNING
        self.feedback_message = "TF available and valid"
        return Status.SUCCESS


class ScanTag(BBBehaviour):
    """Waits for a new detection from AprilTag detector.
       If nothing detected within `timeout_sec` seconds, mission ended."""

    def __init__(self, name="ScanTag", timeout_sec=10.0):
        super().__init__(
            name,
            reads=["tag_seq", "tag_last_id"],
            writes=["target_tag_id", "mission_end_reason"],
        )
        self.timeout = Duration(seconds=timeout_sec)
        self._start, self._seq0 = None, 0

    def initialise(self):
        self._start = self.node.get_clock().now()
        self._seq0 = self.bb_get("tag_seq", 0)
        self.node.get_logger().info("Scanning package's AprilTag for info...")

    def update(self):
        if self.bb_get("tag_seq", 0) > self._seq0:
            tag_id = self.bb_get("tag_last_id")
            self.bb.set("target_tag_id", tag_id)
            self.node.get_logger().info(f"AprilTag scanned: Tag ID {tag_id}")
            self.feedback_message = f"tag {tag_id}"
            return Status.SUCCESS
        if (self.node.get_clock().now() - self._start) > self.timeout:
            self.bb.set("mission_end_reason", "no_more_packages")
            self.node.get_logger().error("Timeout: No AprilTag detected at storage location.")
            return Status.FAILURE
        return Status.RUNNING


class ResolveDeliveryPose(BBBehaviour):
    """Write delivery position coordinates on BB from detected target_tag_id."""

    def __init__(self, name="ResolveDeliveryPose"):
        super().__init__(name, reads=["target_tag_id"], writes=["delivery_pose"])

    def update(self):
        tag_id = self.bb_get("target_tag_id")
        pose = pose_from_params(self.node, f"delivery_locations.tag_{tag_id}")
        if pose is None:
            self.node.get_logger().error(f"No delivery position found for tag_id {tag_id}.")
            self.feedback_message = f"No delivery zone for tag {tag_id}"
            return Status.FAILURE
        self.bb.set("delivery_pose", pose)
        self.feedback_message = f"Delivery zone correctly found for {tag_id}"
        return Status.SUCCESS


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


# ==========================================================================
# Build tree
# ==========================================================================
def set_fork(name, height):
    def make_request(_bb):
        req = SetForkHeight.Request()
        req.height = float(height)
        return req
    return ServiceCall(name, SetForkHeight, "/set_fork_height", make_request)


def attach(name, value):
    def make_request(_bb):
        req = ToggleAttachment.Request()
        req.attach = bool(value)
        return req
    return ServiceCall(name, ToggleAttachment, "/toggle_attachment", make_request)


def navigate(name, make_goal, reads=()):
    return ActionCall(
        name, NavigateToPose, "navigate_to_pose", make_goal, reads=reads,
        format_feedback=lambda fb: f"{fb.feedback.distance_remaining:.2f} m remaining",
    )


def get_tag_position():
    def make_request(bb):
        req = GetTagPosition.Request()
        req.tag_id = int(bb.get("target_tag_id"))
        return req

    def on_response(bb, res):
        if not res.found:
            return False
        bb.set("tag_position", res.position)
        return True

    return ServiceCall("GetTagPosition", GetTagPosition, "/apriltag_detection/get_tag_position",
                       make_request, on_response, reads=["target_tag_id"], writes=["tag_position"])


def parse_tag(msg: PointStamped):
    """Il tag id in header.frame_id as '<...>:=:<id>'."""
    frame = msg.header.frame_id
    if ":=:" not in frame:
        return None
    try:
        return {"tag_last_id": int(frame.split(":=:")[1])}
    except (ValueError, IndexError):
        return None


def create_root(tf_buffer, storage_pose, dock_pose):

    # Write perceptions from topics to blackboard
    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=False)
    topics2bb.add_children([
        TopicToBlackboard("Battery2BB", "/battery_state", BatteryState, ["battery_percentage"],
                          lambda m: {"battery_percentage": m.percentage}),
        TopicToBlackboard("Dock2BB", "/dock_status", DockStatus, ["is_docked"],
                          lambda m: {"is_docked": m.is_docked}),
        TopicToBlackboard("Tag2BB", "/apriltag_detection/tag_detection", PointStamped,
                          ["tag_last_id"], parse_tag, count_key="tag_seq"),
    ])

    # Check battery
    # If not ok go to dock station, dock and charge
    charge = py_trees.composites.Sequence(name="Charge", memory=True)
    charge.add_children([
        navigate("GoToDock", lambda _bb: nav_goal(dock_pose)),
        ActionCall("Dock", Dock, "/dock", lambda _bb: Dock.Goal()),
        py_trees.behaviours.Running(name="Charging"),   # until BatteryOK not return SUCCESS
    ])
    battery_manager = py_trees.composites.Selector(name="BatteryManager", memory=False)
    battery_manager.add_children([BatteryOK(), charge])

    # Undock with manual fallback
    # memory=True to not interrupt when is_docked=False (keeps going backwards)
    undock_with_fallback = py_trees.composites.Selector(name="UndockWithFallback", memory=True)
    undock_with_fallback.add_children([
        py_trees.decorators.Timeout(
            name="UndockTimeout", duration=20.0,
            child=ActionCall("Undock", Undock, "/undock", lambda _bb: Undock.Goal()),
        ),
        ManualMove("ManualReverse", linear_x=-0.15, duration_sec=3.0),
    ])
    undock_manager = py_trees.composites.Selector(name="UndockManager", memory=True)
    undock_manager.add_children([IsUndocked(), undock_with_fallback])

    # Pick Up and Delivery
    pick_up = py_trees.composites.Sequence(name="PickUp", memory=True)
    pick_up.add_children([
        set_fork("LowerFork to pick", 0.0),
        ManualMove("ApproachPackage", linear_x=0.1, duration_sec=2.5),
        attach("Attach", True),
        set_fork("LiftFork to pick", 0.10),
        ManualMove("BackFromStorage", linear_x=-0.1, duration_sec=2.0),
    ])
    drop = py_trees.composites.Sequence(name="Drop", memory=True)
    drop.add_children([
        set_fork("LowerFork to drop", 0.0),
        attach("Detach", False),
        ManualMove("BackFromDelivery", linear_x=-0.1, duration_sec=2.0),
        set_fork("LiftFork to drop", 0.10),
    ])

    # Complete Delivery Cycle
    cycle = py_trees.composites.Sequence(name="DeliveryCycle", memory=True)
    cycle.add_children([
        battery_manager,
        undock_manager,
        TFReady("MapTF", tf_buffer, [("map", "base_link")]),
        navigate("GoToStorage", lambda _bb: nav_goal(storage_pose)),
        ScanTag(),
        get_tag_position(),
        ResolveDeliveryPose(),
        pick_up,
        navigate("GoToDelivery", lambda bb: nav_goal(bb.get("delivery_pose")), reads=["delivery_pose"]),
        drop,
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
    #  - Wait for TF and Nav2 Services       #
    # - Build PoseStamped objs for fixed POI #
    # ====================================== #

    rclpy.init(args=args)
    node = rclpy.create_node("supervisor_node", automatically_declare_parameters_from_overrides=True)
    log = node.get_logger()

    log.info("=== BT SUPERVISOR: Bootstrap ===")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    log.info("Waiting for odometry TF (odom -> base_link)...")
    while rclpy.ok() and not tf_buffer.can_transform("odom", "base_link", Time()):
        rclpy.spin_once(node, timeout_sec=0.5)

    initial_pose = pose_from_params(node, "initial_pose")
    if initial_pose is None:
        log.warn("Initial pose not given in YAML. Using default (0.0, 0.0, 0.0)")
        initial_pose = make_pose(0.0, 0.0, 0.0)

    storage_pose = pose_from_params(node, "storage_location")
    if storage_pose is None:
        log.warn("Storage location not given in YAML. Using default (5.0, 0.0, 0.0)")
        storage_pose = make_pose(5.0, 0.0, 0.0)

    dock_pose = pose_from_params(node, "dock_approach")
    if dock_pose is None:
        log.warn("dock_approach not given in YAML. Using initial_pose as dock approach pose")
        dock_pose = initial_pose

    navigator = BasicNavigator()
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    navigator.setInitialPose(initial_pose)
    log.info("Waiting for Nav2 stack to become ACTIVE...")
    navigator.waitUntilNav2Active()
    navigator.destroy_node()          # Nav2 only used via action node

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