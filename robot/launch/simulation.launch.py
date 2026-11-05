import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, package_dir, launch_dir):
    mode = LaunchConfiguration('mode').perform(context).lower()
    joypad = LaunchConfiguration('joypad').perform(context).lower() == 'true'
    world_name = LaunchConfiguration('world').perform(context)

    # Resolve the world path.
    # Accepts: warehouse, warehouse.sdf, or an absolute path.
    if os.path.isabs(world_name):
        world_path = world_name
        if not world_path.endswith('.sdf'):
            world_path += '.sdf'
    else:
        if not world_name.endswith('.sdf'):
            world_name += '.sdf'

        world_path = os.path.join(package_dir, 'worlds', world_name)

    if not os.path.isfile(world_path):
        raise FileNotFoundError(
            f"World file not found: {world_path}"
        )

    actions = []

    # SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam.launch.py')
        )
    )

    # navigation
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation.launch.py')
        )
    )

    # RViz for slam / odom (nav has its own rviz config)
    rviz_custom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz.launch.py')
        ),
        launch_arguments={'mode': mode}.items(),
    )

    # Joypad control
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'joy_control.launch.py')
        )
    ) 

    if joypad: actions.append(joy_launch)
    if mode == 'slam': 
        actions.append(slam_launch)
        actions.append(rviz_custom)
    elif mode == 'odom':
        actions.append(rviz_custom)
    elif mode == 'nav':
        actions.append(nav2_launch)
        pass

    return actions + [
        # Essentials
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'essentials.launch.py')
            )
        ),
        
        # ROS-Gazebo bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'bridge.launch.py')
            )
        ),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        ),

        # Drop Action Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('drop_action'),
+                             'launch', 'drop_action_server.launch.py')
            )
        ),

        # Spawn robot after Gazebo starts
        TimerAction(
            period=1.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'spawn_robot.launch.py')
                    ),
                    launch_arguments={
                        'x': LaunchConfiguration('x'),
                        'y': LaunchConfiguration('y'),
                        'z': LaunchConfiguration('z'),
                    }.items(),
                )
            ],
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('robot')
    launch_dir = os.path.join(package_dir, 'launch')


    filter_params_file = os.path.join(
        package_dir,
        'config',
        'scan_filter.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='warehouse_world',
            description=(
                'World name (e.g. warehouse), filename (warehouse.sdf), '
                'or absolute path to an SDF file'
            ),
        ),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.2'),
        DeclareLaunchArgument(
            'mode',
            default_value='slam',
            description='Mode of running the simulation. Either slam/odom/nav',
        ),
        DeclareLaunchArgument(
            'joypad',
            default_value='true',
            description='Enable Joypad control',
        ),
        OpaqueFunction(
            function=launch_setup,
            kwargs={
                'package_dir': package_dir,
                'launch_dir': launch_dir,
            },
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter_chain',
            parameters=[filter_params_file],
            remappings=[
                ('scan', '/scan_raw'),
                ('scan_filtered', '/scan'),
            ],
            output='screen',
        ),
    ])
