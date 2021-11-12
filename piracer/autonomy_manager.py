from cmd_msgs.msg import Command

from autonomy_mgmt_msgs.srv import Enable

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node

from transitions import Machine


class AutonomyManager(Node):
    """Create a subscriber, publisher, service client, and state machine for autonomous control.
    """
    def __init__(self):
        super().__init__('autonomy_manager')

        self._agent_name = self.get_namespace().lstrip('/')
        self.timer_counter = 0
        self._direction = True

        self._init_params()
        self._init_pub()
        self._init_sub()
        self._init_srv()

        self._mode_machine = ModeSwitchingMachine(self)
        self._init_mode_machine()

        self._driving_machine = DrivingMachine(self)
        self._init_driving_machine()

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

        # Behavior services
        self.straight_behavior_client = self.create_client(Enable, 'straight_behavior_enable_service')
        while not self.straight_behavior_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("straight_behavior_enable_service not available, waiting...")

        self.behavior_request = Enable.Request()

        self.arc_behavior_client = self.create_client(Enable, 'arc_behavior_enable_service')
        while not self.arc_behavior_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("arc_behavior_enable_service not available, waiting...")

        # Behavior parameter-setting services
        self.straight_param_client = self.create_client(SetParameters, 'straight_behavior/set_parameters')
        while not self.straight_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("straight_behavior/set_parameters not available, waiting...")

        self.behavior_param_request = SetParameters.Request()

        self.arc_param_client = self.create_client(SetParameters, 'arc_behavior/set_parameters')
        while not self.arc_behavior_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("arc_behavior/set_parameters not available, waiting...")

    def _init_mode_machine(self):
        states = ['auto', 'direct', 'experiment']
        transitions = [
            {'trigger': 'direct', 'source': '*', 'dest': 'direct'},
            {'trigger': 'auto', 'source': '*', 'dest': 'auto'},
            {'trigger': 'experiment', 'source': '*', 'dest': 'experiment'}
        ]
        machine = Machine(model=self._mode_machine, states=states, transitions=transitions, initial='direct')

    def _init_driving_machine(self):
        states = ['stop', 'straight', 'arc']
        transitions = [
            {'trigger': 'stop', 'source': '*', 'dest': 'stop'},
            {'trigger': 'straight', 'source': '*', 'dest': 'straight'},
            {'trigger': 'arc', 'source': '*', 'dest': 'arc'}
        ]
        machine = Machine(model=self._driving_machine, states=states, transitions=transitions, initial='stop')

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

    def experiment_timer(self):
        self.timer_counter += 1
        if self.timer_counter % 2 == 0:
            if self._direction:
                self.update_arc_params(-.75, -90.0)
            else:
                self.update_arc_params(.75, 90.0)
            self._direction = not self._direction
            self._driving_machine.arc()
        else:
            self._driving_machine.stop()

    def update_straight_params(self, velocity):
        parameter = Parameter()
        parameter.name = 'velocity'
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = velocity
        self.behavior_param_request.parameters.append(parameter)

        self.future = self.straight_param_client.call_async(self.behavior_param_request)

    def update_arc_velocity(self, velocity):
        parameter = Parameter()
        parameter.name = 'velocity'
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = velocity
        self.behavior_param_request.parameters.append(parameter)

        self.future = self.arc_param_client.call_async(self.behavior_param_request)

    def update_arc_steering_angle(self, velocity):
        parameter = Parameter()
        parameter.name = 'steering_angle'
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = velocity
        self.behavior_param_request.parameters.append(parameter)

        self.future = self.arc_param_client.call_async(self.behavior_param_request)

    def update_arc_params(self, velocity, steering_angle):
        self.update_arc_velocity(velocity)
        self.update_arc_steering_angle(steering_angle)


class ModeSwitchingMachine:
    def __init__(self, autonomy_manager):
        self._autonomy_manager = autonomy_manager
        self._agent_name = self._autonomy_manager._agent_name

    # Direct mode   -----------------------------
    def on_enter_direct(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now entering DIRECT mode.")
        self._autonomy_manager.enable_ackermann()

    def on_exit_direct(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now exiting DIRECT mode.")
        self._autonomy_manager.disable_ackermann()

    # Auto mode     -----------------------------
    def on_enter_auto(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now entering AUTO mode.")

    def on_exit_auto(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now exiting AUTO mode.")

    # Experiment mode   -------------------------
    def on_enter_experiment(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now entering EXPERIMENT mode.")
        self._autonomy_manager.timer = self._autonomy_manager.create_timer(
            1, self._autonomy_manager.experiment_timer
        )

    def on_exit_experiment(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now exiting EXPERIMENT mode.")
        self._autonomy_manager.timer.destroy()
        self._autonomy_manager._driving_machine.stop()


class DrivingMachine:
    def __init__(self, autonomy_manager):
        self._autonomy_manager = autonomy_manager
        self._agent_name = self._autonomy_manager._agent_name

    # STOP  -------------------------------------
    def on_enter_stop(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now entering STOP mode.")

    def on_exit_stop(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now exiting STOP mode.")

    # STRAIGHT  ---------------------------------
    def on_enter_straight(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now entering STRAIGHT mode.")
        self._autonomy_manager.behavior_request.enable = True
        self._autonomy_manager.future = self._autonomy_manager.straight_behavior_client.call_async(
            self._autonomy_manager.behavior_request)

    def on_exit_straight(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now exiting STRAIGHT mode.")
        self._autonomy_manager.behavior_request.enable = False
        self._autonomy_manager.future = self._autonomy_manager.straight_behavior_client.call_async(
            self._autonomy_manager.behavior_request)

    # ARC   -------------------------------------
    def on_enter_arc(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now entering ARC mode.")
        self._autonomy_manager.behavior_request.enable = True
        self._autonomy_manager.future = self._autonomy_manager.arc_behavior_client.call_async(
            self._autonomy_manager.behavior_request)

    def on_exit_arc(self):
        self._autonomy_manager.get_logger().info(f"{self._agent_name} is now exiting ARC mode.")
        self._autonomy_manager.behavior_request.enable = False
        self._autonomy_manager.future = self._autonomy_manager.arc_behavior_client.call_async(
            self._autonomy_manager.behavior_request)


def main():
    """Boilerplate ROS node spinup."""
    rclpy.init()
    node = AutonomyManager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
