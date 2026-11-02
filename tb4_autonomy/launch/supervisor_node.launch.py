import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_autonomy = get_package_share_directory('tb4_autonomy')
    config_file = os.path.join(pkg_autonomy, 'config', 'positions.yaml')

    supervisor_node = Node(
        package='tb4_autonomy',
        executable='supervisor_node',
        name='supervisor_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        supervisor_node
    ])