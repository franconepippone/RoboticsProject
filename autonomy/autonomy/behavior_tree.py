#!/usr/bin/env python3
import tf2_ros
import py_trees
from rclpy.duration import Duration
from nav2_msgs.action import DockRobot, UndockRobot
from sensor_msgs.msg import BatteryState
from rclpy.time import Time

from custom_interfaces.action import PickupPallet, GoTo, DropPackage 
from custom_interfaces.srv import SelectTag 

from autonomy.helpers import make_pose
from autonomy.bt_generic_blocks import BBBehaviour, TopicToBlackboard, ActionCall, SETUP_WAIT_SEC

Status = py_trees.common.Status
Access = py_trees.common.Access

BATTERY_LOW = 0.30      # Threshold to check when starting a delivery cycle
BATTERY_FULL = 1.00     # Threshold to consider the robot charged

N_ZONES = 4

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
            self.feedback_message = "Waiting for /battery_state. Docking determined by current [A] sign."
            return Status.RUNNING
        return Status.FAILURE if docked else Status.SUCCESS

class TFReady(py_trees.behaviour.Behaviour):
    """RUNNING until (target, source) TF are available.
       Creates its own TF buffer/listener in setup() using the node passed by the tree."""
 
    def __init__(self, name, pairs):
        super().__init__(name)
        self.pairs = pairs
        self.buffer, self._listener = None, None
 
    def setup(self, **kwargs):
        node = kwargs["node"]
        self.buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self.buffer, node)
 
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
class ScanTag(BBBehaviour):
    """Calls /select_tag until a tag is visible. After `timeout_sec` without tags: mission ends."""

    def __init__(self, name="ScanTag", timeout_sec=10.0, retry_sec=1.0):
        super().__init__(name, writes=["target_tag_id", "delivery_zone", "mission_end_reason"])
        self.timeout, self.retry = Duration(seconds=timeout_sec), Duration(seconds=retry_sec)
        self.client, self._future, self._start, self._last = None, None, None, None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.client = self.node.create_client(SelectTag, "/select_tag")
        if not self.client.wait_for_service(timeout_sec=SETUP_WAIT_SEC):
            raise RuntimeError(f"[{self.name}] service '/select_tag' not available")

    def initialise(self):
        self._future, self._last = None, None
        self._start = self.node.get_clock().now()
        self.node.get_logger().info("Scanning packages AprilTags for info...")

    def update(self):
        now = self.node.get_clock().now()

        if self._future is None:
            if self._last is None or now - self._last >= self.retry:
                self._future, self._last = self.client.call_async(SelectTag.Request()), now
            return Status.RUNNING
        
        if not self._future.done():
            return Status.RUNNING
        
        res, self._future = self._future.result(), None
        if res.success:
            self.bb.set("target_tag_id", res.tag_id)
            self.bb.set("delivery_zone", f"delivery_zone_{res.tag_id % N_ZONES}")
            self.feedback_message = f"Tag {res.tag_id} selected"
            return Status.SUCCESS
        
        if now - self._start >= self.timeout:
            self.bb.set("mission_end_reason", "no_more_packages")
            self.feedback_message = "No tag visible"
            return Status.FAILURE
        
        return Status.RUNNING

class IncrementCounter(BBBehaviour):
    """Increments the slot counter of `delivery_zone` once a package has been dropped."""

    def __init__(self, name="IncrementCounter"):
        super().__init__(name, reads=["delivery_zone", "delivery_counters"], writes=["delivery_counters"])

    def update(self):
        zone = self.bb_get("delivery_zone")
        counters = self.bb_get("delivery_counters")
        if zone is None or counters is None:
            return Status.FAILURE
        counters = dict(counters)       # new dict so the blackboard sees a write
        counters[zone] += 1
        self.bb.set("delivery_counters", counters)
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

# ========== #
# Build Tree #
# ========== #
def create_root():

    # Slot counters per delivery zone
    init = py_trees.blackboard.Client(name="BTInit")
    init.register_key("delivery_counters", access=Access.WRITE)
    init.delivery_counters = {f"delivery_zone_{i}": 0 for i in range(N_ZONES)}

    # Write perceptions from topics to blackboard
    topics2bb = py_trees.composites.Sequence(name="Topics2BB", memory=False)
    topics2bb.add_children([
        TopicToBlackboard(
            "Battery2BB", "/battery_state", BatteryState, ["battery_percentage", "is_docked"],
            lambda m: {
                "battery_percentage": m.percentage,
                "is_docked": m.power_supply_status in (BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                                                       BatteryState.POWER_SUPPLY_STATUS_FULL),
            }),
    ])

    # Check Battery
    # If not ok go to dock station, dock and charge
    charge = py_trees.composites.Sequence(name="Charge", memory=True)
    dock = ActionCall("Dock", DockRobot, "/dock_robot",
                      lambda _bb: DockRobot.Goal(use_dock_id=True, dock_id="home_dock",
                                                 navigate_to_staging_pose=True))
    
    dock_manager = py_trees.composites.Selector(name="DockManager", memory=True) # memory=True: IsDocked is not re-evaluated
    dock_manager.add_children([py_trees.decorators.Inverter(name="IsDocked", child=IsUndocked()), dock])
    charge.add_children([
        dock_manager,
        py_trees.behaviours.Running(name="Charging"),   # until BatteryOK not return SUCCESS
    ])
    battery_manager = py_trees.composites.Selector(name="BatteryManager", memory=False)
    battery_manager.add_children([BatteryOK(), charge])

    # Undock
    undock = ActionCall("Undock", UndockRobot, "/undock_robot",
                        lambda _bb: UndockRobot.Goal(dock_type="charging_dock"))
    undock_manager = py_trees.composites.Selector(name="UndockManager", memory=False)
    undock_manager.add_children([IsUndocked(), undock])

    # Go to storage, scand tags to choose package and pick one
    pickup_package = py_trees.composites.Sequence(name="PickupPackage", memory=True)
    pickup_package.add_children([
        ActionCall("GoToStorage", GoTo, "/go_to",
                   lambda _bb: GoTo.Goal(poi_name="storage_location", slot_index=0),
                   format_feedback=lambda fb: fb.feedback.status),
        ScanTag(),
        ActionCall("PickupPackage", PickupPallet, "/pickup_pallet",
                   lambda bb: PickupPallet.Goal(tag_id=bb.get("target_tag_id")),
                   reads=["target_tag_id"]),
    ])

    # Go to delivery zone and drop package
    deliver_package = py_trees.composites.Sequence(name="DeliverPackage", memory=True)
    deliver_package.add_children([
        ActionCall("GoToDeliveryZone", GoTo, "/go_to",
                   lambda bb: GoTo.Goal(poi_name=bb.get("delivery_zone"),
                                        slot_index=bb.get("delivery_counters")[bb.get("delivery_zone")]),
                   reads=["delivery_zone", "delivery_counters"]),
        ActionCall("DropPackage", DropPackage, "/drop_package", lambda _bb: DropPackage.Goal()),
        IncrementCounter(),
    ])

    # Complete Delivery Cycle
    cycle = py_trees.composites.Sequence(name="DeliveryCycle", memory=True)
    cycle.add_children([
        battery_manager,
        undock_manager,
        TFReady("MapTF", [("map", "base_link")]),
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