
from geometry_msgs.msg import PoseStamped, TransformStamped

from scipy.spatial.transform import Rotation

import math

from geometry_msgs.msg import PoseStamped, TransformStamped


def compute_staging_pose(
    tag_tf: TransformStamped,
    offset: float = 0.8,
) -> PoseStamped:
    """Compute the staging pose for this pallet docking session from the april tag tf.
    Offset is how far away from the tag the staging pose is in meters."""

    pose = PoseStamped()
    q = tag_tf.transform.rotation

    # Rotate tag's local +Z axis into odom.
    tag_normal_x = 2.0 * (q.x * q.z + q.w * q.y)
    tag_normal_y = 2.0 * (q.y * q.z - q.w * q.x)

    # Position offset along tag normal.
    pose.pose.position.x = (
        tag_tf.transform.translation.x + offset * tag_normal_x
    )
    pose.pose.position.y = (
        tag_tf.transform.translation.y + offset * tag_normal_y
    )
    pose.pose.position.z = 0.0

    # Robot +X points opposite to tag +Z.
    yaw = math.atan2(-tag_normal_y, -tag_normal_x)

    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose