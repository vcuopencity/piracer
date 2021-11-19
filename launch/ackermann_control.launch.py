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
    car_config = join(get_package_share_directory('piracer'),
                      'config', 'car_config.yaml')
    piracer_launch_directory = get_package_share_directory('piracer')
    bridge_launch_directory = get_package_share_directory('mqtt_bridge')

    launch_hardware = LaunchConfiguration('launch_hardware')
    launch_bridge = LaunchConfiguration('launch_bridge')

    # noinspection PyTypeChecker
    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_name',
            default_value='car1',
            description='Sets the namespace for this car.'
        ),
        DeclareLaunchArgument(
            'launch_hardware',
            default_value='true',
            description='Determines if hardware_nodes.launch is called.'
        ),
        DeclareLaunchArgument(
            'launch_bridge',
            default_value='true',
            description='Determinds if ackermann_drive_bridge.launch is called from the mqtt_bridge package.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/hardware_nodes.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name')
            }.items(),
            condition=IfCondition(launch_hardware)
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='ackermann_controller',
            name='ackermann_controller',
            parameters=[car_config],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bridge_launch_directory, '/launch/ackermann_drive_bridge.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name')
            }.items(),
            condition=IfCondition(launch_bridge)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bridge_launch_directory, '/launch/twist_bridge.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name')
            }.items(),
            condition=IfCondition(launch_bridge)
        )
    ])
