import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    default_config = os.path.join(
        get_package_share_directory('battery_node'), 'config', 'parameters.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='YAML file with the battery_node parameters')

    battery_node = Node(
        package='battery_node',
        executable='battery_node',
        name='battery_node',
        parameters=[LaunchConfiguration('config_file'), {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([config_arg, battery_node])