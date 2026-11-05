from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params = os.path.join(get_package_share_directory('drive_mode'), 'config', 'drive_mode.yaml')

    drive_mode_node = Node(
        package='drive_mode',  
        executable='drive_mode',  
        name='drive_mode',
        parameters=[params],     
        output='screen'            
    )

    return LaunchDescription([
            drive_mode_node
    ])