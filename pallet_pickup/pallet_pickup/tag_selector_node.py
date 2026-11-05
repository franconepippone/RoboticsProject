import time
import copy

from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.service import Service

from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from custom_interfaces.srv import SelectTag

import random

@dataclass(frozen=True)
class TagCandidate:
    tag_id: int


def select_tag_policy(candidates: List[TagCandidate]) -> Optional[int]:
    """
    Select a tag from the currently visible candidates.

    Returns:
        Selected tag ID, or None if no tag is selected.
    """

    # Default policy: choose a random tag
    return random.choice([c.tag_id for c in candidates])




@dataclass
class TrackedDetection:
    tag_id: int
    last_seen: int



class AprilTagSelector(Node):

    def __init__(self) -> None:
        super().__init__('april_tag_selector')

        self.declare_parameter('visibility_timeout', 0.5)

        visibility_timeout_seconds: float = self.get_parameter('visibility_timeout').get_parameter_value().double_value
        self.visibility_timeout_ns = int(visibility_timeout_seconds * 1_000_000_000)

        if self.visibility_timeout_ns <= 0:
            raise ValueError('visibility timeout must be positive')

        # Currently visible tags cache, indexed by tag ID.
        self.visible_tags: Dict[int, TrackedDetection] = {}

        # detections published by ros_apriltags
        self.subscription: Subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )

        # Selection service
        self.selection_service: Service = self.create_service(
            SelectTag,
            '/select_tag',
            self.select_tag_callback
        )

        self.get_logger().info('AprilTag selector initialized')

    def detection_callback(
        self,
        msg: AprilTagDetectionArray
    ) -> None:

        now: int = self.get_clock().now().nanoseconds
        
        for detection in msg.detections:
            detection: AprilTagDetection
            tag_id: int = detection.id

            if not isinstance(tag_id, int) or tag_id < 0:
                self.get_logger().warning(f'Received an invalid tag id from april tag detection: {tag_id}')
                continue

            # adds or overwrites 
            self.visible_tags[tag_id] = TrackedDetection(
                tag_id=tag_id,
                last_seen=now
                # NOTE multiple info can be added here
            )

        self.expire_old_detections()

    def expire_old_detections(self) -> None:
        """
        Removes all expired tags from memory (forgets expired tags)
        """

        now: int = self.get_clock().now().nanoseconds

        expired_ids: List[int] = [
            tag_id
            for tag_id, tracked in self.visible_tags.items()
            if now - tracked.last_seen > self.visibility_timeout_ns
        ]

        self.get_logger().debug('Expiring tags with ids: ' + ', '.join(str(i) for i in expired_ids))

        for tag_id in expired_ids:
            del self.visible_tags[tag_id]


    def select_tag_callback(
        self,
        request: SelectTag.Request,
        response: SelectTag.Response
    ) -> SelectTag.Response:

        # first remove expired tags
        self.expire_old_detections()

        candidates: List[TagCandidate] = [
            TagCandidate(tag_id=tag_id)
            for tag_id in sorted(self.visible_tags.keys())
        ]

        self.get_logger().info('Select tag service called, visible tag candidates ids: ' + ', '.join(str(c.tag_id) for c in candidates))

        if not candidates:
            return self.build_failed_response(response, 'No AprilTags currently visible')

        try:
            selected_id: Optional[int] = select_tag_policy(candidates)

        except Exception as e:
            self.get_logger().error(f'Selection policy raised: {e}')
            return self.build_failed_response(response, 'Selection policy failed')

        available_ids: set[int] = {
            candidate.tag_id
            for candidate in candidates
        }

        self.get_logger().info(f'Tag policy returned tag ID: {selected_id}')

        # assert the selected id is actually in the candidates
        if selected_id is None or selected_id not in available_ids:
            self.get_logger().warning(f'Tag ID selected by policy is None or unavailable: {selected_id}')
            return self.build_failed_response(response, 'Policy returned an invalid tag ID')

        response.success = True
        response.tag_id = selected_id
        response.message = 'Tag selected successfully'

        self.get_logger().info(f'Selected tag: {selected_id}')
        return response

    @staticmethod
    def build_failed_response(response: SelectTag.Response, message: str) -> SelectTag.Response:
        response.success = False
        response.tag_id = -1
        response.message = message
        return response


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: AprilTagSelector = AprilTagSelector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()