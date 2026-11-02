import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Define parameters for simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get the absolute paths for the required packages
    tb4_pkg = get_package_share_directory('turtlebot4_gz_bringup')
    forklift_desc_pkg = get_package_share_directory('forklift_description')
    forklift_mgr_pkg = get_package_share_directory('forklift_manager')
    april_tag_pkg = get_package_share_directory('april_tag_detection')
    tb4_autonomy_pkg = get_package_share_directory('tb4_autonomy')

    # Append environment variable for resources
    append_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(forklift_mgr_pkg, 'models')
    )

    # Declare SLAM argument
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Starts SLAM toolbox with the simulation'
    )

    # Declare Nav2 argument
    declare_nav2_cmd = DeclareLaunchArgument(
        'nav2',
        default_value='false',
        description='Starts Nav2 navigation stack'
    )

    # Declare AMCL argument
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Starts AMCL node for localization'
    )

    # Declare map argument
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Absolute path to .yaml file of a map'
    )

    # Launch TurtleBot 4
    tb4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_pkg, 'launch', 'turtlebot4_gz.launch.py')
        ),
        launch_arguments=[
            ('rviz', 'true'),
            ('slam', LaunchConfiguration('slam')),
            ('nav2', LaunchConfiguration('nav2')),
            ('localization', LaunchConfiguration('localization')),
            ('map', LaunchConfiguration('map')),
            ('use_sim_time', use_sim_time)
        ]
    )

    # Launch AprilTag Detection System with 10s delay
    april_tag_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(april_tag_pkg, 'launch', 'start_detection.launch.py')
                ),
                launch_arguments=[('use_sim_time', use_sim_time)]
            )
        ]
    )

    # Spawn Forklift
    # Delayed by 5 seconds to allow Gazebo to fully load the world
    spawn_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(forklift_desc_pkg, 'launch', 'spawn_forklift.launch.py')
                ),
                launch_arguments=[('use_sim_time', use_sim_time)]
            )
        ]
    )

    # Create forklift_bridge 
    # Delayed by 7 seconds to ensure Gazebo topics are available for mapping
    bridge_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(forklift_mgr_pkg, 'launch', 'forklift_bridge.launch.py')
                ),
                launch_arguments=[('use_sim_time', use_sim_time)]
            )
        ]
    )

    # Create Forklift Manager Node
    manager_node = Node(
        package='forklift_manager',
        executable='forklift_manager_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Spawn test package contemporarily with forklift
    spawn_package_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'test_package',
                    '-file', os.path.join(forklift_mgr_pkg, 'models', 'test_package', 'test_package.sdf'),
                    '-x', '5.5', '-y', '0.0', '-z', '0.1'
                ],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    ) 

    # Detach command
    # Delayed by 10 seconds to ensure models and junctions are already loaded
    detach_package_cmd = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'topic', '-t', '/forklift_fork/package/detach', '-m', 'gz.msgs.Empty', '-p', ' '],
                output='screen'
            )
        ]
    )

    # Start supervisor node for turtlebot autonomy
    start_supervisor = TimerAction(
        period= 40.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tb4_autonomy_pkg, 'launch', 'supervisor_node.launch.py')
                ),
                launch_arguments=[('use_sim_time', use_sim_time)]
            )
        ]
    )

    return LaunchDescription([
        declare_slam_cmd,
        declare_nav2_cmd,
        declare_localization_cmd,
        declare_map_cmd,
        tb4_launch,
        april_tag_launch,
        spawn_launch,
        bridge_launch,
        manager_node,
        spawn_package_node,
        detach_package_cmd,
        start_supervisor,
        append_gz_resource_path
    ])