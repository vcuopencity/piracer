# Standard library imports
from os.path import join
from os import environ

# Third-party imports
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """ Lowest-level launch for piracer, including only the nodes that interact directly with the hardware.
    """
    default_config = join(get_package_share_directory('piracer'),
                      'config', 'default_config.yaml')
                    
    agent_name = "car" + environ['CAR_ID']

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Agent configuration .yaml file.'
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='steering_driver',
            name='steering_driver',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='throttle_driver',
            name='throttle_driver'
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='display_driver',
            name='display_driver'
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='power_monitor_driver',
            name='power_monitor_driver'
        ),
        Node(
            package='bno055_driver',
            namespace=agent_name,
            executable='bno055_driver',
            name='bno055_driver',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])