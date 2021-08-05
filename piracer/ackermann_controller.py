from ackermann_msgs.msg import AckermannDrive

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class AckermannController(Node):
    """Create a pub/sub for the ackermann drive bridge in the mqtt_bridge package, create publishers for the hardware
    nodes, create a service server for the autonomy_manager to control whether or not ackermann control is active.
    """
    def __init__(self):
        super().__init__('ackermann_driver')

        # Parameters
        self.declare_parameter('ackermann_bridge_input_topic', 'ros_pub_top')
        self.input_topic = self.get_parameter('ackermann_bridge_input_topic').get_parameter_value().string_value

        self.declare_parameter('ackermann_bridge_output_topic', 'ros_sub_top')
        self.output_topic = self.get_parameter('ackermann_bridge_output_topic').get_parameter_value().string_value

        #  Bridge subscriber / publisher
        self.create_ackermann_subscription()

        self.ackermann_pub = self.create_publisher(
            msg_type=AckermannDrive,
            topic=self.output_topic,
            qos_profile=10,
        )

        # Publishers for hardware nodes
        self.steering_pub = self.create_publisher(
            msg_type=Float64,
            topic='angle',
            qos_profile=10,
        )
        self.throttle_pub = self.create_publisher(
            msg_type=Float64,
            topic='throttle',
            qos_profile=10,
        )

    def create_ackermann_subscription(self):
        """Subscribe to the ackermann drive bridge."""
        self.ackermann_sub = self.create_subscription(
            msg_type=AckermannDrive,
            topic=self.input_topic,
            callback=self.ackermann_callback,
            qos_profile=10,
        )

    def ackermann_callback(self, msg):
        """Publish speed and steering angle to their appropriate topics."""
        self.get_logger().debug(
            f'RECEIVED | angle: {msg.steering_angle:.2f} '
            f'angle_vel: {msg.steering_angle_velocity:.2f} '
            f'speed: {msg.speed:.2f} '
            f'accel: {msg.acceleration:.2f} '
            f'jerk: {msg.jerk:.2f} '
        )

        throttle_msg = Float64()
        throttle_msg.data = msg.speed

        steer_msg = Float64()
        steer_msg.data = msg.steering_angle

        self.throttle_pub.publish(throttle_msg)
        self.steering_pub.publish(steer_msg)

        self.ackermann_pub.publish(msg)


def main():
    """Boilerplate ROS node spinup."""
    rclpy.init()
    node = AckermannController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
