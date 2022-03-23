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
    """ Launching the PiRacer for the demo with IMU information publishing over the bridge.
    """
    car_config = join(get_package_share_directory('piracer'),
                      'config', 'car_config.yaml')
    bridge_launch_directory = get_package_share_directory('mqtt_bridge')
    launch_bridge = LaunchConfiguration('launch_bridge')
    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_name',
            default_value='car1',
            description='Sets the namespace for this car.'),
        DeclareLaunchArgument(
            'launch_bridge',
            default_value='true',
            description='Determinds if appropriate bridge files are launched from the mqtt_bridge package.'
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='display_driver',
            name='display_driver'
        ),
        Node(
            package='bno055_driver',
            namespace=[LaunchConfiguration('agent_name')],
            executable='bno055_driver',
            name='bno055_driver',
            parameters=[car_config]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bridge_launch_directory, '/launch/imu_bridge.launch.py']),
            launch_arguments={
                'agent_name': LaunchConfiguration('agent_name')
            }.items(),
            condition=IfCondition(launch_bridge)
        )
    ])
