import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    sdf_file = os.path.join(package_dir, 'description', 'model.sdf')

    with open(sdf_file, 'r') as f:
        robot_description = f.read()

    twist_mux_params = os.path.join(package_dir, 'config', 'twist_mux.yaml')
    drive_mode_params = os.path.join(package_dir, 'config', 'drive_mode.yaml')
    
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
        
        Node(
            package='robot',
            executable='drive_mode',
            name='drive_mode',
            parameters=[drive_mode_params],
            output='screen',
        ),

        # used to mix joypad and navigation cmds
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_params],
            output='screen'
        )
    ])