from cmd_msgs.msg import Command

from autonomy_mgmt_msgs.srv import Enable

import rclpy
from rclpy.node import Node

from transitions import Machine


class AutonomyManager(Node):
    """Create a subscriber, publisher, service client, and state machine for autonomous control.
    """
    def __init__(self):
        super().__init__('autonomy_manager')

        # Topic parameterd
        self.declare_parameter('bridge_input_topic', 'control_input_topic')
        input_topic = self.get_parameter('bridge_input_topic').get_parameter_value().string_value

        self.declare_parameter('bridge_output_topic', 'control_output_topic')
        output_topic = self.get_parameter('bridge_output_topic').get_parameter_value().string_value

        # Bridge subscriber / publisher
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

        # State machine
        states = ['auto', 'direct']
        transitions = [
            {'trigger': 'direct', 'source': '*', 'dest': 'direct'},
            {'trigger': 'auto', 'source': '*', 'dest': 'auto'}
        ]
        machine = Machine(model=self, states=states, transitions=transitions, initial='direct')

        # Service client
        self.enable_ackermann_client = self.create_client(Enable, 'ackermann_bridge_enable_service')
        while not self.enable_ackermann_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("ackermann_bridge_enable_service not available, waiting...")
        self.ack_request = Enable.Request()

    # State Machine callbacks ---------------------------------------------------------------------
    # Direct mode
    def on_enter_direct(self):
        self.get_logger().info("Car is now entering DIRECT mode.")
        self.enable_ackermann()

    def on_exit_direct(self):
        self.get_logger().info("Car is now exiting DIRECT mode.")
        self.disable_ackermann()

    # Auto mode
    def on_enter_auto(self):
        self.get_logger().info("Car is now entering AUTO mode.")

    def on_exit_auto(self):
        self.get_logger().info("Car is now exiting AUTO mode.")

    # Service callbacks ---------------------------------------------------------------------------
    def enable_ackermann(self):
        self.ack_request.enable = True
        self.future = self.enable_ackermann_client.call_async(self.ack_request)

    def disable_ackermann(self):
        self.ack_request.enable = False
        self.future = self.enable_ackermann_client.call_async(self.ack_request)

    # Subscriber callbacks ------------------------------------------------------------------------
    def command_callback(self, msg):
        """Responding to a received command."""
        self.command_pub.publish(msg)

        if msg.operational_mode.lower() == 'direct':
            if self.state == 'direct':
                self.get_logger().info("Car is already in DIRECT mode!")
            else:
                self.direct()
        if msg.operational_mode.lower() == 'auto':
            if self.state == 'auto':
                self.get_logger().info("Car is already in AUTO mode!")
            else:
                self.auto()


def main():
    """Boilerplate ROS node spinup."""
    rclpy.init()
    node = AutonomyManager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
