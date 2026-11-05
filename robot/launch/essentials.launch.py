import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    sdf_file = os.path.join(package_dir, 'description', 'model.sdf')

    with open(sdf_file, 'r') as f:
        robot_description = f.read()

    twist_mux_params = os.path.join(package_dir, 'config', 'twist_mux.yaml')
    drive_mode_launch = os.path.join(
        get_package_share_directory('drive_mode'), 'launch', 'drive_mode_node.launch.py')
    
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

        IncludeLaunchDescription(PythonLaunchDescriptionSource(drive_mode_launch)),
    ])