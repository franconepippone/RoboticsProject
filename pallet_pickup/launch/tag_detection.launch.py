from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('pallet_pickup')
    params_file = os.path.join(package_share, 'config', 'apriltag.yaml')

    # launches apriltag_ros detection node with appropriate remappings and paramters from config file
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[params_file],
    )

    # launches static transform publisher to convert camera_link frame to camera_optical_frame,
    # needed to properly align the apriltag detections tf with the camera frame
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_transform',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '-1.570796327',
            '--pitch', '0',
            '--yaw', '-1.570796327',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_optical_frame',
        ],
        output='screen',
    )

    return LaunchDescription([
        camera_transform,
        apriltag_node,
    ])