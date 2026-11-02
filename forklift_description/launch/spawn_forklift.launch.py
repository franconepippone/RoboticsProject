import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # Retrieve the path to the package installation directory
    pkg_share = get_package_share_directory('forklift_description')
    sdf_file = os.path.join(pkg_share, 'models', 'forklift_fork', 'model.sdf')

    # Append environment variable to instruct Gazebo where to find the "models" directory
    set_gz_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, '..') 
    )

    # Define node to spawn the model in Gazebo
    spawn_forklift = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file,
            '-name', 'forklift_fork',
            # Spawn the model at the origin; the DetachableJoint plugin will snap it to the robot
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # Delay the spawn by 5 seconds to give TurtleBot4 time to fully load
    # Ensures correct order of spawn
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_forklift]
    )

    # Define node for TF tree continuity
    # Assume zero offset between base_link and the root frame of the forklift,
    # since the DetachableJoint physically merges the two frames.
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0', 
            '--frame-id', 'base_link',
            '--child-frame-id', 'mast'],
        output='screen'
    )

    return LaunchDescription([
        set_gz_model_path,
        delayed_spawn,
        static_tf_publisher
    ])