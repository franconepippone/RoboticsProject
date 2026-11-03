from contextlib import contextmanager
from typing import Iterator

from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

class DockPoseBridge:
    """Publishes the selected AprilTag pose in a target frame while active.
    Use as a component inside a master node.
    
    Provides 'publish_for' for context manager usage"""

    def __init__(
        self,
        node: Node,
        target_frame: str = 'odom',
        tag_frame_prefix: str = 'tag36h11:',
        output_topic: str = '/detected_dock_pose',
        publish_rate: float = 10.0,
        max_detection_age: float = 1.5,
    ):
        self.node = node
        self.target_frame = target_frame
        self.tag_frame_prefix = tag_frame_prefix
        self.max_detection_age = max_detection_age

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, node)

        self.publisher = node.create_publisher(
            PoseStamped, output_topic, 10
        )

        self.target_tag_id: int | None = None
        self.active = False

        # TODO make the timer inactive if bridge is also inactive
        self.timer = node.create_timer(
            1.0 / publish_rate, self._publish_pose,
        )
    
    @contextmanager
    def publish_for(self, tag_id: int) -> Iterator[None]:
        """Publish the selected tag pose for the duration of the context."""
        self.start(tag_id)
        try:
            yield
        finally:
            self.stop()

    def start(self, tag_id: int) -> None:
        """Start publishing the pose of the selected tag."""
        if tag_id < 0:
            raise ValueError('tag_id must be non-negative')

        self.target_tag_id = tag_id
        self.active = True

        self.node.get_logger().info(
            f'Dock pose bridge started for tag {tag_id}'
        )

    def stop(self) -> None:
        """Stop publishing dock poses."""
        self.active = False
        self.target_tag_id = None

        self.node.get_logger().info('Dock pose bridge stopped')

    def _publish_pose(self) -> None:
        if not self.active or self.target_tag_id is None:
            return

        tag_frame = f'{self.tag_frame_prefix}{self.target_tag_id}'

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                tag_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )

            stamp = Time.from_msg(transform.header.stamp)
            age = (
                self.node.get_clock().now() - stamp
            ).nanoseconds / 1e9

            if age < 0.0 or age > self.max_detection_age:
                self.node.get_logger().warn(
                    f'Transform for tag {self.target_tag_id} is too old or invalid (age: {age:.3f}s)'
                )
                return

            msg = PoseStamped()
            msg.header = transform.header
            msg.header.frame_id = self.target_frame

            msg.pose.position.x = transform.transform.translation.x
            msg.pose.position.y = transform.transform.translation.y
            msg.pose.position.z = transform.transform.translation.z

            # we let the docking server re-orient this pose (via config yaml) so that its aligned with the robot frame convention
            # Right now, this is z forward (towards the robot, normal to tag), x right (seen from bot), y up.
            # what we want is z up, x forward, y left (as in robot base link)
            msg.pose.orientation = transform.transform.rotation

            self.publisher.publish(msg)

        except Exception as e:
            # Tag not currently available in TF.
            self.node.get_logger().error(f'Error occurred while looking up transform: {e}')
            return
        


if __name__ == '__main__':

    class myNode(Node):
        def __init__(self):
            super().__init__('my_node')
            self.dock_pose_bridge = DockPoseBridge(self)

            # When starting docking:
            self.dock_pose_bridge.start(tag_id=3)

            # When docking finishes or fails:
            self.dock_pose_bridge.stop()