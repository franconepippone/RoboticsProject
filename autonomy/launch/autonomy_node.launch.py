import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('autonomy'), 'config', 'parameters.yaml')
    return LaunchDescription([
        Node(package='autonomy', executable='autonomy_node', name='autonomy_node',
             parameters=[config, {'use_sim_time': True}], output='screen'),
    ])