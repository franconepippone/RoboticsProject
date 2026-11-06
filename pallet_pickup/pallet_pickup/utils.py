import numpy as np
from rclpy.node import Node
from rclpy.task import Future
from rclpy.callback_groups import CallbackGroup

from geometry_msgs.msg import PoseStamped, TransformStamped
import math

from geometry_msgs.msg import PoseStamped, TransformStamped, Transform


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


def get_yaw_error(tag_transform: Transform) -> float:
    """Compute the yaw error between the tag's orientation and the robot's orientation.
    
    'tag_transform' must be the raw april tag transform in the robot's base_link frame.
    The transform's z axis must point orthogonal to the tag's surface, and the robot 'forward'
    must be the x axis.

    Since the tag points towards the robot during docking, to get true alignment the z axis versor is first flipped,
    the projected onto the x-y plane and then the angle is measured.

    The returned angle is in [-pi, pi] radians.
    """
    
    q = tag_transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w

    # B's Z axis expressed in A, flipped
    vx = -2.0 * (x*z + w*y)
    vy = -2.0 * (y*z - w*x)

    if np.hypot(vx, vy) < 1e-6:
        raise ValueError("Projected vector is undefined")

    return np.arctan2(vy, vx)




async def asleep(
    node: Node,
    seconds: float,
    cb_group: CallbackGroup | None = None
) -> None:
    future: Future = Future()

    def wakeup() -> None:
        if not future.done():
            future.set_result(None)

    timer = node.create_timer(
        seconds,
        wakeup,
        callback_group=cb_group,
    )

    try:
        await future
    finally:
        node.destroy_timer(timer)