# Standard library imports
from os.path import join
from os import environ

# Third-party imports
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch hardware_nodes.launch and the teleop_control node for teleop control of the piracer.
    """
    default_config = join(get_package_share_directory('piracer'),
                      'config', 'default_config.yaml')
    launch_directory = get_package_share_directory('piracer')
    launch_ackermann = LaunchConfiguration('launch_ackermann')

    agent_name = "car" + environ['CAR_ID']

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_ackermann',
            default_value='True',
            description='Determines if ackermann_control.launch is called.'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Agent configuration .yaml file.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_directory, '/ackermann_control.launch.py']),
            launch_arguments={
                'agent_name': agent_name
            }.items(),
            condition=IfCondition(launch_ackermann)
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='straight_behavior',
            name='straight_behavior',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='arc_behavior',
            name='arc_behavior',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])
