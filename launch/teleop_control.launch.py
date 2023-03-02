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
    launch_directory = get_package_share_directory('piracer')
    default_config = join(get_package_share_directory('piracer'),
                      'config', 'default_config.yaml')
    launch_hardware = LaunchConfiguration('launch_hardware')
    agent_name = "car" + environ['CAR_ID']

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_hardware',
            default_value='True',
            description='Determines if hardware_nodes.launch is called.'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Agent configuration .yaml file.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_directory, '/hardware_nodes.launch.py']),
            launch_arguments={
                'config_file' : LaunchConfiguration('config_file')
            }.items(),
            condition=IfCondition(launch_hardware)
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='teleop_controller',
            name='teleop_controller'
        )
    ])
