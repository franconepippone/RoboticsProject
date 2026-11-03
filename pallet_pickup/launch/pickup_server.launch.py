from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('pallet_pickup')

    # AprilTag detection stack launch
    tag_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                'launch',
                'tag_detection.launch.py'
            )
        )
    )

    # LiDAR obstacle detector for determining succesfully package alignment and pickup
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
            )
        ]
    )

    return LaunchDescription([
        tag_detection_launch,
        obstacle_detector,
    ])