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

def pose_from_params(node, prefix: str):
    """PoseStamped from <prefix>.x/.y/.yaw in ROS parameters.
       None if parameters do not exist."""
    if not node.has_parameter(f"{prefix}.x"):
        return None
    return make_pose(
        node.get_parameter(f"{prefix}.x").value,
        node.get_parameter(f"{prefix}.y").value,
        node.get_parameter(f"{prefix}.yaw").value,
    )

# ========== #
# Nav Helper #
# ========== #
def nav_goal(pose: PoseStamped) -> NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    goal.pose = pose
    return goal