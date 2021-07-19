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
    """ Launch hardware_nodes.launch and autonomy_manager node, as well as every control mode of the piracer using each
    respective launch file, passing launch_hardware=false so there aren't duplicates of the hardware nodes running.
    """
    command_bridge_topics = join(get_package_share_directory('piracer'),
                                 'config', 'command_bridge_topics.yaml')
    piracer_launch_directory = get_package_share_directory('piracer')
    bridge_launch_directory = get_package_share_directory('mqtt_bridge')
    launch_bridge = LaunchConfiguration('launch_bridge')
    return LaunchDescription([
        DeclareLaunchArgument(
            'car_name',
            default_value='car1',
            description='Sets the namespace for this car.'),
        DeclareLaunchArgument(
            'launch_bridge',
            default_value='true',
            description='Determinds if ackermann_drive_bridge.launch is called from the mqtt_bridge package.'
        ),
        IncludeLaunchDescription(

            PythonLaunchDescriptionSource([piracer_launch_directory, '/ackermann_control.launch.py']),
            launch_arguments={
                'car_name': LaunchConfiguration('car_name'),
                'launch_hardware': 'false'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/teleop_control.launch.py']),
            launch_arguments={
                'car_name': LaunchConfiguration('car_name'),
                'launch_hardware': 'false'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/hardware_nodes.launch.py']),
            launch_arguments={
                'car_name': LaunchConfiguration('car_name')
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bridge_launch_directory, '/launch/command_bridge.launch.py']),
            launch_arguments={
                'car_name': LaunchConfiguration('car_name')
            }.items(),
            condition=IfCondition(launch_bridge)
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='autonomy_manager',
            name='autonomy_manager',
            parameters=[command_bridge_topics]
        )
    ])
