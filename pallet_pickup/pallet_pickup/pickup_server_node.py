from typing import cast
import math 

import rclpy
from rclpy.impl.logging_severity import LoggingSeverity
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
from pallet_pickup.control_utils.odometric_planar_controller import OdometricPlanarController
from pallet_pickup.utils import compute_staging_pose, get_yaw_error

from robot.forklift_api import ForkliftAPI

class PickupServer(Node):

    def __init__(self):
        super().__init__('pickup_server')

        self.pose_bridge = DockPoseBridge(
            node=self, 
            max_detection_age=5.0           # allowing a bigger age allows docking to recover even if tag is temporarily hidden
        )
        self.tag_cache = TagTFCache(
            node=self, 
            target_frame='odom'            # fixed frame of docking
        )
        self.robot_controller = OdometricPlanarController(
            node=self,
            cmd_topic='/cmd_vel',
            odom_topic='/odom',
            frequency=20,

            # drift coefficients (odom_movement : actual_movement ratio)
            drift_linear_coeff=1.0,
            drift_rot_coeff=.7,     # TODO: testing on this revealed a value close to 1 in sim. Why why need it this small here? Problem is prob somewhere else
        )
        self.forklift_api = ForkliftAPI(self)

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

        # shorthand for aborting the goal
        def _abort(msg: str) -> PickupPallet.Result:
            self.get_logger().error(msg)
            goal_handle.abort()
            return self._build_abort_response(msg)

        self.get_logger().info('Received pallet docking request')

        ### 0. Verify dependencies are available

        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            return _abort('Nav2 docking server unavailable')


        ### 1. Validate tag ID and if tag ID is detected available and stable

        self.get_logger().info('Validating tag ID and availability')

        request: PickupPallet.Goal = goal_handle.request
        tag_id = request.tag_id

        if not isinstance(tag_id, int) or tag_id < 0:
            return _abort(f'Invalid tag ID: {tag_id}')
        
        transform = self.tag_cache.get_tag_frame(tag_id, 5.0) 
        if transform is None:
            return _abort(f'Tag not available or too old: {tag_id}')
        
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
                return _abort('Nav2 rejected docking goal')

            result_future: Future = nav_goal_handle.get_result_async()
            nav_result: DockRobot_GetResult_Response = cast(DockRobot_GetResult_Response, await result_future)

        # outside ctxmng
        self.get_logger().debug('Dock pose bridge disabled for tag {tag_id}')

        if not nav_result.result.success:
            return _abort(f'Nav2 docking failed: {nav_result.result.error_msg} [{nav_result.result.error_code}]')


        ### 3. Fine tune yaw alignment manually

        self.get_logger().info('Fine tuning yaw aligment with respect to tag')

        transform = self.tag_cache.get_tag_frame(
            tag_id, 
            max_age=5.0,
            target_frame_override='base_link',  # we want the tag in base_link to compute yaw error 
            timeout=5.0
        )

        if transform is None:
            return _abort(f'Tag is undetectable for yaw fine tuning: {tag_id}')
        
        error_rad = get_yaw_error(transform.transform)
        self.get_logger().info(f'Measured yaw error after docking of: {error_rad} radians')

        # rotate by the leftover aligment error after docking completes, to try to compensate final yaw error
        # TODO this is not very reliable, probably due to a bad error_rad value
        await self.robot_controller.rotate_by(error_rad)

        self.get_logger().info(f'Rotation terminated, angle should have been compensated.')


        ### 4. If success, move forward to place forklift underneath pallet
        self.get_logger().info(f'Moving robot underneath the pallet.')

        DISTANCE = .27 #m

        await self.robot_controller.forward_by(DISTANCE)
    
        ### 5. Lift forklift
        self.get_logger().info(f'Lifting forklift.')

        self.forklift_api.lift()

        ### 6. Assert package has been picked up

        # TODO async wait for a bit

        await self.robot_controller.forward_by(-.5)

        result = PickupPallet.Result()
        result.success = True
        result.message = str(nav_result.result.error_msg)

        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)

    node = PickupServer()
    node.get_logger().set_level(LoggingSeverity.DEBUG)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()