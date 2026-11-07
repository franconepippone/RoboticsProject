import copy
import math
import time
import yaml
import rclpy
from typing import Dict, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future

from custom_interfaces.action import GoTo

DELIVERY_PREFIX = 'delivery_zone_'   # keys inside `delivery_locations`, e.g. delivery_zone_1
DELIVERY_CONTAINER = 'delivery_locations'


class GoToServer(Node):

    def __init__(self):
        super().__init__('go_to_action_server')

        # Parameters
        self.declare_parameter('poi_file', '')               # path of poi.yaml (required)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('staging_distance', 1.0)      # m between staging pose and drop pose
        self.declare_parameter('slot_spacing', 1.0)          # m between two consecutive packages in a zone
        self.declare_parameter('max_slots_per_zone', 4)

        self.map_frame = self.get_parameter('map_frame').value
        self.staging_distance = float(self.get_parameter('staging_distance').value)
        self.slot_spacing = float(self.get_parameter('slot_spacing').value)
        self.max_slots = int(self.get_parameter('max_slots_per_zone').value)

        poi_file = self.get_parameter('poi_file').value
        if not poi_file: raise RuntimeError("Parameter 'poi_file' is required")

        # POI Poses
        # poi_poses: dock_location, storage_location, ...
        # delivery_zone_poses: drop pose of the FIRST slot of every delivery zone
        self.poi_poses: Dict[str, PoseStamped]
        self.delivery_zone_poses: Dict[str, PoseStamped]
        self.poi_poses, self.delivery_zone_poses = self._load_poi_file(poi_file)

        self.get_logger().info(f'Loaded POIs: {list(self.poi_poses)}')
        self.get_logger().info(f'Loaded delivery zones: {list(self.delivery_zone_poses)}')

        # Nav2 Client
        self.cb_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self, NavigateToPose,
            self.get_parameter('nav_action_name').value,
            callback_group=self.cb_group,
        )

        # Action Server
        self._busy = False
        self._action_server = ActionServer(
            self,
            GoTo,
            'go_to',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('GoTo Server waiting for requests...')

    # ======================= #
    # Action Server Callbacks #
    # ======================= #
    def goal_callback(self, _) -> GoalResponse:
        if self._busy:
            self.get_logger().warning('Goal rejected: another GoTo is in progress')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _) -> CancelResponse:
        self.get_logger().info('Cancellation requested')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self._busy = True
        try: return await self._execute(goal_handle)
        finally: self._busy = False

    async def _execute(self, goal_handle: ServerGoalHandle):
        poi_name = goal_handle.request.poi_name
        slot_index = goal_handle.request.slot_index
        feedback_msg = GoTo.Feedback()

        def finish(outcome: str, message: str) -> GoTo.Result:
            result = GoTo.Result()
            result.success = (outcome == 'ok')
            result.message = message
            if outcome == 'ok':
                goal_handle.succeed()
                self.get_logger().info(message)
            elif outcome == 'canceled':
                goal_handle.canceled()
                self.get_logger().warning(message)
            else:
                goal_handle.abort()
                self.get_logger().error(message)
            return result

        self.get_logger().info(f"Received request to go to '{poi_name}'")

        # Determine goal pose
        if poi_name in self.poi_poses: goal_pose = copy.deepcopy(self.poi_poses[poi_name])

        elif poi_name in self.delivery_zone_poses:

            if not 0 <= slot_index < self.max_slots: return finish('failed', f'Invalid slot {slot_index} for {poi_name} (max {self.max_slots})')

            feedback_msg.status = f'Computing staging pose (package #{slot_index} of {poi_name})...'
            self.get_logger().info(feedback_msg.status)
            goal_handle.publish_feedback(feedback_msg)
            goal_pose = self._compute_staging_pose(poi_name, slot_index)

        else: return finish('failed', f"Unknown point of interest: '{poi_name}'")

        # Navigate with Nav2
        feedback_msg.status = 'Moving to goal pose...'
        goal_handle.publish_feedback(feedback_msg)
        outcome, message = await self._navigate(goal_handle, goal_pose, feedback_msg)

        return finish(outcome, message)

    # ================================================================== #
    #                            Utilities                               #
    # ================================================================== #

    # ==== #
    # Nav2 #
    # ==== #
    async def _navigate(self, goal_handle: ServerGoalHandle, pose: PoseStamped,
                        feedback_msg: GoTo.Feedback) -> Tuple[str, str]:
        """Send pose to Nav2 and wait. Returns (outcome, message), outcome in ok|failed|canceled."""

        if not self.nav_client.wait_for_server(timeout_sec=5.0): return 'failed', 'Nav2 NavigateToPose server unavailable'

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()

        last_fb = [0.0]

        def on_feedback(fb_msg):
            now = time.monotonic()
            if now - last_fb[0] < 1.0: return # forward at most 1 Hz
            last_fb[0] = now
            feedback_msg.status = f'Navigating: {fb_msg.feedback.distance_remaining:.2f} m remaining'
            goal_handle.publish_feedback(feedback_msg)

        nav_handle = await self.nav_client.send_goal_async(nav_goal, feedback_callback=on_feedback)
        if not nav_handle.accepted: return 'failed', 'Nav2 rejected the goal'

        result_future = nav_handle.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                await nav_handle.cancel_goal_async()
                return 'canceled', 'GoTo canceled, Nav2 goal canceled'
            await self._sleep(0.1)

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED: return 'ok', 'Goal pose reached correctly!'
        return 'failed', f'Nav2 navigation ended with status {status}'

    async def _sleep(self, seconds: float) -> None:
        """Non-blocking sleep that follows the node clock (sim time)."""
        
        done = Future()

        def fire():
            if not done.done():
                done.set_result(None)

        timer = self.create_timer(seconds, fire, callback_group=self.cb_group)
        try: await done
        finally: self.destroy_timer(timer)

    # ============ #
    # YAML Parsing #
    # ============ #
    def _load_poi_file(self, path: str) -> Tuple[Dict[str, PoseStamped], Dict[str, PoseStamped]]:
        
        with open(path, 'r') as f: data = yaml.safe_load(f)

        # ROS params file: {'/**': {'ros__parameters': {...}}} -> take the first node entry
        params = next(iter(data.values()))['ros__parameters']

        pois: Dict[str, PoseStamped] = {}
        zones: Dict[str, PoseStamped] = {}

        for name, entry in params.items():
            if name == DELIVERY_CONTAINER:
                for zone_name, zone_entry in entry.items():
                    zones[zone_name] = self._pose_from_dict(zone_entry)
            else: pois[name] = self._pose_from_dict(entry)

        return pois, zones

    def _pose_from_dict(self, d: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pos, ori = d['position'], d['orientation']
        pose.pose.position.x = float(pos['x'])
        pose.pose.position.y = float(pos['y'])
        pose.pose.position.z = float(pos.get('z', 0.0))
        pose.pose.orientation.x = float(ori['x'])
        pose.pose.orientation.y = float(ori['y'])
        pose.pose.orientation.z = float(ori['z'])
        pose.pose.orientation.w = float(ori['w'])
        return pose

    # ========================= #
    # Delivery Pose Computation #
    # ========================= #
    def _compute_staging_pose(self, zone_name: str, n:int) -> PoseStamped:
        """Staging pose for the next free slot of `zone_name`."""

        zone_num = int(zone_name[len(DELIVERY_PREFIX):])

        # deepcopy, otherwise stored pose would be modified
        pose = copy.deepcopy(self.delivery_zone_poses[zone_name])
        q = pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w) # planar orientation

        # Drop pose of slot n: zones 1,2 shift towards +x, zones 3,4 towards -x
        direction = 1.0 if zone_num in (1, 2) else -1.0
        pose.pose.position.x += direction * n * self.slot_spacing

        # Staging pose: `staging_distance` BEHIND the drop pose, along the drop heading
        pose.pose.position.x -= self.staging_distance * math.cos(yaw)
        pose.pose.position.y -= self.staging_distance * math.sin(yaw)
        
        return pose


def main(args=None):
    rclpy.init(args=args)
    go_to_node = GoToServer()

    try:
        rclpy.spin(go_to_node)
    except KeyboardInterrupt:
        pass
    finally:
        go_to_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()