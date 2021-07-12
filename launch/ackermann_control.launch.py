# Standard library imports
from os.path import join

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
    """Launch hardware_nodes.launch and the ackermann_control node for ackermann drive control of the piracer. Intended
    for use with the ackermann_drive.launch configuration of the mqtt_bridge package.
    """
    bridge_topics = join(get_package_share_directory('piracer'),
                         'config', 'ackermann_bridge_topics.yaml')
    launch_directory = get_package_share_directory('piracer')
    launch_hardware = LaunchConfiguration('launch_hardware')

    return LaunchDescription([
        DeclareLaunchArgument(
            'car_name',
            default_value='car1',
            description='Sets the namespace for this car.'
        ),
        DeclareLaunchArgument(
            'launch_hardware',
            default_value='true',
            description='Determines if hardware_nodes.launch is called.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_directory, '/hardware_nodes.launch.py']),
            launch_arguments={
                'car_name': LaunchConfiguration('car_name')
            }.items(),
            condition=IfCondition(launch_hardware)
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='ackermann_controller',
            name='ackermann_controller',
            parameters=[bridge_topics],
        )
    ])
