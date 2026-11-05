from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    drop_action_node = Node(
        package='drop_action',  
        executable='drop_action_server',  
        name='drop_action_server',        
        output='screen'            
    )

    return LaunchDescription([
            drop_action_node
    ])