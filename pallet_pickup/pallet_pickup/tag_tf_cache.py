from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

class TagTFCache:
    """Looks up the latest AprilTag transform in a target frame."""

    def __init__(
        self,
        node: Node,
        tag_frame_prefix: str = 'tag36h11:',
        target_frame: str = 'odom',
    ):
        self.node = node
        self.tag_frame_prefix = tag_frame_prefix
        self.target_frame = target_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

        self.node.get_logger().info(
            f'TagTFCache initialized: target_frame={target_frame}'
        )

    def get_tag_frame(
        self,
        tag_id: int,
        max_age: float,
    ) -> TransformStamped | None:
        """
        Return the latest tag transform in target_frame if it is fresh.

        Returns None if the transform is unavailable or too old.
        """
        if tag_id < 0:
            raise ValueError('tag_id must be non-negative')
        if max_age < 0.0:
            raise ValueError('max_age must be non-negative')

        tag_frame = f'{self.tag_frame_prefix}{tag_id}'

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                tag_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as ex:
            self.node.get_logger().debug(
                f'Tag {tag_id} unavailable in {self.target_frame}: {ex}'
            )
            return None

        stamp = Time.from_msg(
            transform.header.stamp,
            clock_type=self.node.get_clock().clock_type,
        )

        age = (
            self.node.get_clock().now() - stamp
        ).nanoseconds / 1e9

        if age < 0.0 or age > max_age:
            self.node.get_logger().debug(
                f'Tag {tag_id} transform is stale: age={age:.3f}s'
            )
            return None

        return transform

    def is_tag_available(self, tag_id: int) -> bool:
        """Placeholder for future implementation."""
        raise NotImplementedError(
            'is_tag_available() has not been implemented yet'
        )