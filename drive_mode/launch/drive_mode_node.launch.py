from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    drive_mode_node = Node(
        package='drive_mode',  
        executable='drive_mode',  
        name='drive_mode_node',        
        output='screen'            
    )

    return LaunchDescription([
            drive_mode_node
    ])