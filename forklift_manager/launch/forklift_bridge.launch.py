import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('forklift_manager')
    bridge_config_file = os.path.join(pkg_share, 'config', 'forklift_bridge.yaml')

    # Node to run the parameter bridge mapping Gazebo topics to ROS 2
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_file}'],
        output='screen'
    )

    return LaunchDescription([
        parameter_bridge
    ])