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
    """ Launch hardware_nodes.launch and autonomy_manager node, as well as every control mode of the piracer using each
    respective launch file, passing launch_hardware=false so there aren't duplicates of the hardware nodes running.
    """
    default_config = join(get_package_share_directory('piracer'),
                      'config', 'default_config.yaml')
    piracer_launch_directory = get_package_share_directory('piracer')
    bridge_launch_directory = get_package_share_directory('mqtt_bridge')
    launch_bridge = LaunchConfiguration('launch_bridge')

    agent_name = "car" + environ['CAR_ID']

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_bridge',
            default_value='true',
            description='Determines if appropriate bridge files are launched from the mqtt_bridge package.'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Agent configuration .yaml file.'
        ),
        IncludeLaunchDescription(

            PythonLaunchDescriptionSource([piracer_launch_directory, '/ackermann_control.launch.py']),
            launch_arguments={
                'agent_name': agent_name,
                'launch_hardware': 'false',
                'config_file' : LaunchConfiguration('config_file')
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/teleop_control.launch.py']),
            launch_arguments={
                'agent_name': agent_name,
                'launch_hardware': 'false',
                'config_file' : LaunchConfiguration('config_file')
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/open_loop_control.launch.py']),
            launch_arguments={
                'agent_name': agent_name,
                'launch_ackermann': 'false',
                'config_file' : LaunchConfiguration('config_file')
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/hardware_nodes.launch.py']),
            launch_arguments={
                'agent_name': agent_name,
                'config_file' : LaunchConfiguration('config_file')
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bridge_launch_directory, '/launch/command_bridge.launch.py']),
            launch_arguments={
                'agent_name': agent_name
            }.items(),
            condition=IfCondition(launch_bridge)
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='autonomy_manager',
            name='autonomy_manager',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='v2x_node',
            name='v2x_node',
            parameters=[LaunchConfiguration('config_file')]
        ),
        Node(
            package='piracer',
            namespace=agent_name,
            executable='vive_driver',
            name='vive_driver',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])
