from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bridge_topics = os.path.join(get_package_share_directory('piracer'),'config','ackermann_bridge_topics.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('car_name', default_value='car1', description='Sets the namespace for this car.'),
        Node(
            package='piracer',
            namespace=[LaunchConfiguration('car_name')],
            executable='steering_driver',
            name='steering_driver'
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
            executable='ackermann_controller',
            name='ackermann_controller',
            parameters=[bridge_topics]
            # parameters=[
            #     {'bridge_input_topic': 'ros_pub_top'},
            #     {'bridge_output_topic': 'ros_sub_top'}
            # ]
        )
    ])
