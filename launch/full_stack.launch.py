# Standard library imports
from os.path import join

# Third-party imports
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """ Launch hardware_nodes.launch and autonomy_manager node, as well as every control mode of the piracer using each
    respective launch file, passing launch_hardware=false so there aren't duplicates of the hardware nodes running.
    """
    command_bridge_topics = join(get_package_share_directory('piracer'),
                                 'config', 'command_bridge_topics.yaml')
    launch_directory = get_package_share_directory('piracer')
    return LaunchDescription([
        DeclareLaunchArgument('car_name', default_value='car1',
                              description='Sets the namespace for this car.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_directory, '/ackermann_control.launch.py']),
            launch_arguments={
                'car_name': LaunchConfiguration('car_name'),
                'launch_hardware': 'false'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_directory, '/teleop_control.launch.py']),
            launch_arguments={
                'car_name': LaunchConfiguration('car_name'),
                'launch_hardware': 'false'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_directory, '/hardware_nodes.launch.py']),
            launch_arguments={
                'car_name': LaunchConfiguration('car_name')
            }.items(),
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='autonomy_manager',
            name='autonomy_manager',
            parameters=[command_bridge_topics]
        )
    ])
