import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    poi_file = os.path.join(get_package_share_directory('robot'), 'config', 'poi.yaml')

    go_to_action_node = Node(
        package='go_to_action',  
        executable='go_to_action_server',  
        name='go_to_action_server',
        parameters=[{'poi_file': poi_file,'use_sim_time': True}],      
        output='screen'            
    )

    return LaunchDescription([
            go_to_action_node
    ])