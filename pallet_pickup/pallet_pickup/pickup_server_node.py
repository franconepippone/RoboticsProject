from typing import cast

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from nav2_msgs.action import DockRobot
from nav2_msgs.action._dock_robot import DockRobot_GetResult_Response
from custom_interfaces.action import PickupPallet

from pallet_pickup.dock_pose_bridge import DockPoseBridge
from pallet_pickup.tag_tf_cache import TagTFCache
from pallet_pickup.utils import compute_staging_pose


class PickupServer(Node):

    def __init__(self):
        super().__init__('pickup_server')

        self.pose_bridge = DockPoseBridge(node=self, max_detection_age=5.0)
        self.tag_cache = TagTFCache(node=self)

        self.callback_group = ReentrantCallbackGroup()

        self.dock_client = ActionClient(
            self,
            DockRobot,
            '/dock_robot'
        )

        self.action_server = ActionServer(
            self,
            PickupPallet,
            '/pickup_pallet',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Pickup Pallet action server ready')

    @staticmethod
    def _build_abort_response(message: str) -> PickupPallet.Result:
        """Helper function to build a standard abort response for the pickup action server."""
        result = PickupPallet.Result()
        result.success = False
        result.message = message
        return result

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """Async cooroutine coordinating all steps involved in the pallet pickup process."""

        self.get_logger().info('Received pallet docking request')

        ### 0. Verify dependencies are available

        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            return self._build_abort_response('Nav2 docking server unavailable')


        ### 1. Validate tag ID and if tag ID is detected available and stable

        self.get_logger().info('Validating tag ID and availability')

        request: PickupPallet.Goal = goal_handle.request
        tag_id = request.tag_id

        if not isinstance(tag_id, int) or tag_id < 0:
            return self._build_abort_response(f'Invalid tag ID: {tag_id}')
        
        transform = self.tag_cache.get_tag_frame(tag_id, 5.0) 
        if transform is None:
            return self._build_abort_response(f'Tag not available or too old: {tag_id}')

        #if not self.tag_cache.is_tag_available(tag_id):
        #    goal_handle.abort()
        #    result = PickupPallet.Result()
        #    result.success = False
        #    result.message = f'Tag not available: {tag_id}'
        #    return result


        ### 2. Forward docking request to Nav2 docking server + enable dock pose bridge

        self.get_logger().info('Forwarding request to Nav2 docking server')

        goal = DockRobot.Goal()

        goal.use_dock_id = False

        goal.dock_pose = compute_staging_pose(transform, offset=1.0)
        goal.dock_pose.header.frame_id = 'odom'
        goal.dock_pose.header.stamp = self.get_clock().now().to_msg()

        goal.dock_type = 'pallet_dock'
        goal.navigate_to_staging_pose = True # should not be required if we are already near
        goal.max_staging_time = 60.0

        # while docking, keep bridge enabled with context manager
        with self.pose_bridge.publish_for(tag_id):
            self.get_logger().debug('Dock pose bridge enabled for tag {tag_id}')

            send_future = self.dock_client.send_goal_async(goal)
            nav_goal_handle: ClientGoalHandle = cast(ClientGoalHandle, await send_future)

            if not bool(nav_goal_handle.accepted):
                return self._build_abort_response('Nav2 rejected docking goal')

            result_future: Future = nav_goal_handle.get_result_async()
            nav_result: DockRobot_GetResult_Response = cast(DockRobot_GetResult_Response, await result_future)

        self.get_logger().debug('Dock pose bridge disabled for tag {tag_id}')
        # outside ctxmng

        result = PickupPallet.Result()
        result.success = nav_result.result.success
        result.message = str(nav_result.result.error_msg)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result
    
        ### 3. If success, move forward to place forklift underneath pallet

        ### 4. Lift forklift

        ### 5. Assert package has been picked up


def main(args=None):
    rclpy.init(args=args)

    node = PickupServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()