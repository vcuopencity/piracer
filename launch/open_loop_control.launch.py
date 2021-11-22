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
    """Launch hardware_nodes.launch and the teleop_control node for teleop control of the piracer.
    """
    car_config = join(get_package_share_directory('piracer'),
                      'config', 'car_config.yaml')
    launch_directory = get_package_share_directory('piracer')
    launch_ackermann = LaunchConfiguration('launch_ackermann')

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_name',
            default_value='car1',
            description='Sets the namespace for this car.'
        ),
        DeclareLaunchArgument(
            'launch_ackermann',
            default_value='True',
            description='Determines if ackermann_control.launch is called.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_directory, '/ackermann_control.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name')
            }.items(),
            condition=IfCondition(launch_ackermann)
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='straight_behavior',
            name='straight_behavior',
            parameters=[car_config]
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='arc_behavior',
            name='arc_behavior',
            parameters=[car_config]
        )
    ])
