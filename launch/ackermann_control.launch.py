from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piracer',
            namespace='car1',
            executable='steering_driver',
            name='steering_driver'
        ),
        Node(
            package='piracer',
            namespace='car1',
            executable='throttle_driver',
            name='throttle_driver'
        ),
        Node(
            package='piracer',
            namespace='car1',
            executable='display_driver',
            name='display_driver'
        ),
        Node(
            package='piracer',
            namespace='car1',
            executable='power_monitor_driver',
            name='power_monitor_driver'
        ),
        Node(
            package='v4l2_camera',
            namespace='car1',
            executable='v4l2_camera_node',
            name='picamera_driver'
        ),
        Node(
            package='piracer',
            namespace='car1',
            executable='ackermann_driver',
            name='ackermann_driver'
        )
    ])
