import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import (
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    sdf_file = os.path.join(package_dir, 'description', 'model.sdf')

    with open(sdf_file, 'r') as f:
        robot_description = f.read()

    twist_mux_params = os.path.join(package_dir, 'config', 'twist_mux.yaml')
    mode = 2
    # pickup server paths
    pickup_dir = get_package_share_directory('pallet_pickup')
    pickup_server_launch_file = os.path.join(pickup_dir, 'launch', 'pickup_server.launch.py')

    return LaunchDescription([

        # publish robot and joint state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # used to mix joypad and navigation cmds
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_params],
            output='screen'
        ),

        # Pallet pickup server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pickup_server_launch_file),
            launch_arguments={'use_sim_time': 'true'}.items()
        )
    ])