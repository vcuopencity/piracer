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
        )
    ])
