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
    """ Launching the PiRacer for the demo with IMU bridge and joy_teleop control.
    """
    default_config = join(get_package_share_directory('piracer'),
                      'config', 'default_config.yaml')
    bridge_launch_directory = get_package_share_directory('mqtt_bridge')
    piracer_launch_directory = get_package_share_directory('piracer')
    launch_bridge = LaunchConfiguration('launch_bridge')
    launch_teleop = LaunchConfiguration('launch_teleop')

    agent_name = "car" + environ['CAR_ID']

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_bridge',
            default_value='true',
            description='Determines if appropriate bridge files are launched from the mqtt_bridge package.'
        ),
        DeclareLaunchArgument(
            'launch_teleop',
            default_value='true',
            description='Determines if teleop_control is launched.'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Agent configuration .yaml file.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bridge_launch_directory, '/launch/imu_bridge.launch.py']),
            launch_arguments={
                'agent_name': agent_name,
                'config_file' : LaunchConfiguration('config_file')
            }.items(),
            condition=IfCondition(launch_bridge)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([piracer_launch_directory, '/teleop_control.launch.py']),
            launch_arguments={
                'agent_name': agent_name
            }.items(),
            condition=IfCondition(launch_teleop)
        )
    ])
