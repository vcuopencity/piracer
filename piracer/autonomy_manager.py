from command_msgs.msg import Command

import rclpy
from rclpy.node import Node


class AutonomyManager(Node):
    def __init__(self):
        super().__init__('autonomy_manager')
        """Initializing necessary members of this node."""
        # Topic parameters
        self.declare_parameter('bridge_input_topic', 'control_input_topic')
        input_topic = self.get_parameter('bridge_input_topic').get_parameter_value().string_value

        self.declare_parameter('bridge_output_topic', 'control_output_topic')
        output_topic = self.get_parameter('bridge_output_topic').get_parameter_value().string_value

        # Bridge sub / pub
        self.command_sub = self.create_subscription(
            msg_type=Command,
            topic=input_topic,
            callback=self.command_callback,
            qos_profile=10,
        )
        self.command_pub = self.create_publisher(
            msg_type=Command,
            topic=output_topic,
            qos_profile=10,
        )

    def command_callback(self, msg):
        """Responding to a received command."""
        self.command_pub.publish(msg)

        if msg.operational_mode.lower() == 'direct':
            self.get_logger().info("Car is now in DIRECT mode.")
        if msg.operational_mode.lower() == 'auto':
            self.get_logger().info("Car is now in AUTO mode.")


def main():
    """Boilerplate ROS node spinup."""
    rclpy.init()
    node = AutonomyManager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
