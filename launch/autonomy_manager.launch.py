# Standard library imports
from os.path import join

# Third-party imports
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    command_bridge_topics = join(get_package_share_directory('piracer'),
                                 'config', 'command_bridge_topics.yaml')
    ackermann_bridge_topics = join(get_package_share_directory('piracer'),
                                   'config', 'ackermann_bridge_topics.yaml')
    steering_config = join(get_package_share_directory('piracer'),
                           'config', 'steering_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('car_name', default_value='car1',
                              description='Sets the namespace for this car.'),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='steering_driver',
            name='steering_driver',
            parameters=[steering_config]
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='throttle_driver',
            name='throttle_driver'
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='display_driver',
            name='display_driver'
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='power_monitor_driver',
            name='power_monitor_driver'
        ),
        Node(
            package='v4l2_camera',
            namespace=[LaunchConfiguration('car_name')],
            executable='v4l2_camera_node',
            name='picamera_driver'
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='autonomy_manager',
            name='autonomy_manager',
            parameters=[command_bridge_topics]
        ),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='ackermann_controller',
            name='ackermann_controller',
            parameters=[ackermann_bridge_topics]
        )
    ])
