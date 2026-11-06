from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('pallet_pickup')

    # Simulation time configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # AprilTag detection stack launch
    tag_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                'launch',
                'tag_detection.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # LiDAR obstacle detector for determining successfully package alignment and pickup
    obstacle_detector = Node(
        package='pallet_pickup',
        executable='laser_obstacle_detector',
        name='laser_obstacle_detector',
        output='screen',
        parameters=[
            os.path.join(
                package_share,
                'config',
                'obstacle_detector.yaml'
            ),
            {'use_sim_time': use_sim_time}
        ]
    )

    # Master pickup action server
    pickup_server = Node(
        package='pallet_pickup',
        executable='pickup_server',
        name='pickup_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        tag_detection_launch,
        obstacle_detector,
        pickup_server
    ])