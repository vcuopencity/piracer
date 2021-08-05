# Standard library imports
from os.path import join

# Third-party imports
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """ Lowest-level launch for piracer, including only the nodes that interact directly with the hardware.
    """
    car_config = join(get_package_share_directory('piracer'),
                      'config', 'car_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('agent_name', default_value='car1',
                              description='Sets the namespace for this car.'),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='steering_driver',
            name='steering_driver',
            parameters=[car_config]
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='throttle_driver',
            name='throttle_driver'
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='display_driver',
            name='display_driver'
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('agent_name')],
            executable='power_monitor_driver',
            name='power_monitor_driver'
        ),
        Node(
            package='v4l2_camera',
            namespace=[LaunchConfiguration('agent_name')],
            executable='v4l2_camera_node',
            name='picamera_driver'
        )
    ])