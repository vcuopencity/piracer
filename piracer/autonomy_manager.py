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

        self._mode_machine = StateMachine(self)
        self._agent_name = self.get_namespace().lstrip('/')
        agent_name = 'test'

        self._init_params()
        self._init_pub()
        self._init_sub()
        self._init_srv()
        self._init_state_machine()

    def _init_params(self):
        self.declare_parameter('cmd_bridge_input_topic', 'control_input_topic')
        self.cmd_bridge_input_topic = self.get_parameter('cmd_bridge_input_topic').get_parameter_value().string_value

        self.declare_parameter('cmd_bridge_output_topic', 'control_output_topic')
        self.cmd_bridge_output_topic = self.get_parameter('cmd_bridge_output_topic').get_parameter_value().string_value

    def _init_pub(self):
        self.command_pub = self.create_publisher(
            msg_type=Command,
            topic=self.cmd_bridge_output_topic,
            qos_profile=10,
        )

    def _init_sub(self):
        self.command_sub = self.create_subscription(
            msg_type=Command,
            topic=self.cmd_bridge_input_topic,
            callback=self.command_callback,
            qos_profile=10,
        )

    def _init_srv(self):
        self.enable_ackermann_client = self.create_client(Enable, 'ackermann_bridge_enable_service')
        while not self.enable_ackermann_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("ackermann_bridge_enable_service not available, waiting...")
        self.ack_request = Enable.Request()

    def _init_state_machine(self):
        states = ['auto', 'direct', 'experiment']
        transitions = [
            {'trigger': 'direct', 'source': '*', 'dest': 'direct'},
            {'trigger': 'auto', 'source': '*', 'dest': 'auto'},
            {'trigger': 'experiment', 'source': '*', 'dest': 'experiment'}
        ]
        machine = Machine(model=self._mode_machine, states=states, transitions=transitions, initial='direct')

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

        if msg.operational_mode.lower() == self._mode_machine.state:
            self.get_logger().info(f"{self._agent_name} is already in {str(self._mode_machine.state.upper())} mode!")
        elif msg.operational_mode.lower() == 'direct':
            self._mode_machine.direct()
        elif msg.operational_mode.lower() == 'auto':
            self._mode_machine.auto()
        elif msg.operational_mode.lower() == 'experiment':
            self._mode_machine.experiment()


class StateMachine(AutonomyManager):
    # State Machine callbacks ---------------------------------------------------------------------
    # Direct mode
    def __init__(self, autonomy_manager):
        # super().__init__()
        self._agent_name = autonomy_manager.agent_name

    def on_enter_direct(self):
        self.get_logger().info(f"{self._agent_name} is now entering DIRECT mode.")
        self.enable_ackermann()

    def on_exit_direct(self):
        AutonomyManager.get_logger().info(f"{self._agent_name} is now exiting DIRECT mode.")
        AutonomyManager.disable_ackermann()

    # Auto mode
    def on_enter_auto(self):
        self.get_logger().info(f"{self._agent_name} is now entering AUTO mode.")

    def on_exit_auto(self):
        self.get_logger().info(f"{self._agent_name} is now exiting AUTO mode.")

    # Experiment mode
    def on_enter_experiment(self):
        self.get_logger().info(f"{self._agent_name} is now entering EXPERIMENT mode.")

    def on_exit_experiment(self):
        self.get_logger().info(f"{self._agent_name} is now exiting EXPERIMENT mode.")

def main():
    """Boilerplate ROS node spinup."""
    rclpy.init()
    node = AutonomyManager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
