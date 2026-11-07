import math
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# ============ #
# Pose Helpers #
# ============ #
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose