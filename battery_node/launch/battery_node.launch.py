from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    args = [
        DeclareLaunchArgument('publish_rate', default_value='2.0',
                              description='Battery update/publish frequency [Hz]'),
        DeclareLaunchArgument('initial_level', default_value='1.0',
                              description='Initial battery level in [0, 1]'),
        DeclareLaunchArgument('drain_rate', default_value='0.002',
                              description='Discharge rate [1/s]'),
        DeclareLaunchArgument('charge_rate', default_value='0.02',
                              description='Charge rate [1/s]'),
    ]

    battery_node = Node(
        package='battery_node',
        executable='battery_node',
        name='battery_node',
        parameters=[{
            'publish_rate': ParameterValue(LaunchConfiguration('publish_rate'), value_type=float),
            'initial_level': ParameterValue(LaunchConfiguration('initial_level'), value_type=float),
            'drain_rate': ParameterValue(LaunchConfiguration('drain_rate'), value_type=float),
            'charge_rate': ParameterValue(LaunchConfiguration('charge_rate'), value_type=float),
            'use_sim_time': True,
        }],
        output='screen',
    )

    return LaunchDescription(args + [battery_node])