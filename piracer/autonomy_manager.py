from cmd_msgs.msg import Command
from autonomy_mgmt_msgs.srv import Enable

import json
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rosidl_runtime_py import set_message

import paho.mqtt.client
import logging
from transitions import Machine
from transitions import MachineError
from transitions.extensions import GraphMachine
from os import environ


class AutonomyManager(Node):
    """Create a subscriber, publisher, service client, and state machine for autonomous control.
    """
    def __init__(self):
        super().__init__('autonomy_manager')

        self._agent_name = self.get_namespace().lstrip('/')
        self._timer_counter = 0
        self._direction = True
        self._debug_machine = False
        self._client_futures = []

        self._init_params()
        self._init_pub()
        self._init_sub()
        self._init_srv()
        self._init_mqtt()

        self.print_count = 0
        
        self._driving_machine = DrivingMachine(self)
        self._init_driving_machine()

        self._autonomy_machine = AutonomyMachine(self)
        self._init_autonomy_machine()        
        
        self.print_machine("Init")

        if self._debug_machine:
            logging.basicConfig(level=logging.DEBUG)
            # Set transitions' log level to INFO; DEBUG messages will be omitted
            logging.getLogger('transitions').setLevel(logging.INFO)

    def _init_params(self):
        self.declare_parameter('cmd_bridge_input_topic', 'control_input_topic')
        self.cmd_bridge_input_topic = self.get_parameter('cmd_bridge_input_topic').get_parameter_value().string_value

        self.declare_parameter('cmd_bridge_output_topic', 'control_output_topic')
        self.cmd_bridge_output_topic = self.get_parameter('cmd_bridge_output_topic').get_parameter_value().string_value

        self.declare_parameter('initial_mode')
        self.initial_mode = self.get_parameter('initial_mode').get_parameter_value().string_value

        states = ['auto', 'direct', 'experiment']

        if self.initial_mode.lower() not in  states:
            raise ValueError(f"initial_mode must be set to one of {states}")
        
        self.declare_parameter('demo_topic')
        self.demo_topic = self.get_parameter('demo_topic').get_parameter_value().string_value
        if self.demo_topic == '':
            raise ValueError("'demo_topic' parameter not set in config file.")

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
        self.enable_twist_client = self.create_client(Enable, 'twist_bridge_enable_service')
        while not self.enable_twist_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("twist_bridge_enable_service not available, waiting...")

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
    
    def _init_mqtt(self):
        """Create an MQTT client, set callbacks, connect to the broker, and start the loop."""
        self.mqtt_client = paho.mqtt.client.Client(f'/{self._agent_name}/autonomy_manager')

        self.mqtt_client.on_connect = self._mqtt_on_connect
        self.mqtt_client.on_message = self._mqtt_on_message

        if environ.get('MQTT_BROKER_URI') is not None:
            self._mqtt_broker_uri = environ.get('MQTT_BROKER_URI')
        else:
            self._mqtt_broker_uri = "127.0.0.1"
        
        try:
            self.mqtt_client.connect(self._mqtt_broker_uri, 1883)
            self.mqtt_client.loop_start()
        except (ConnectionRefusedError, ValueError):
            self.get_logger().fatal(f"""MQTT Publisher failed to connect to: """
                                    f"""{self._mqtt_broker_uri} Node is terminating...""")
            exit()
    
    def _mqtt_on_connect(self, client, userdata, flags, rc):
        """Log MQTT broker connection code."""
        self.get_logger().info(f"""MQTT Publisher connected to: {self._mqtt_broker_uri}"""
                               f""" with result code: {rc}""")
    
    def _mqtt_on_message(self, client, userdata, msg):
        """Change the state of the vehicle."""
        cmd_dict = json.loads(msg.payload.decode('utf-8'))
        cmd_msg = Command()
        set_message.set_message_fields(cmd_msg, cmd_dict)
        payload = cmd_msg.operational_mode

        # payload = str(msg.payload.decode('utf-8'))
        self.get_logger().debug(f"MQTT message received! {payload}")
        
        try:
            self._driving_machine.trigger(payload)
        except MachineError:
            self.get_logger().error(f"Driving machine cannot transition from {self._driving_machine.state} to {payload}!")
        except AttributeError:
            self.get_logger().error(f"{payload} is not a valid transition!")

    def _init_autonomy_machine(self):
        states = ['initial', 'auto', 'direct', 'experiment']
        transitions = [
            {'trigger': 'direct', 'source': ['initial', 'auto', 'experiment'], 'dest': 'direct'},
            {'trigger': 'auto', 'source': ['initial', 'direct', 'experiment'], 'dest': 'auto'},
            {'trigger': 'experiment', 'source': ['initial', 'auto', 'direct'], 'dest': 'experiment'}
        ]
        machine = GraphMachine(model=self._autonomy_machine, states=states, transitions=transitions, initial='initial')
        # Dummy initial mode is rqd so that "on_enter" callback is called for initial state
        self._autonomy_machine.trigger(self.initial_mode)

    def _init_driving_machine(self):
        states = ['stop', 'straight', 'arc', 'pause_straight', 'pause_arc']
        transitions = [
            {'trigger': 'stop', 'source': ['straight', 'arc', 'pause_straight', 'pause_arc'], 'dest': 'stop'},
            {'trigger': 'straight', 'source': ['arc', 'stop'], 'dest': 'straight'},
            {'trigger': 'arc', 'source': ['straight', 'stop'], 'dest': 'arc'},
            {'trigger': 'pause', 'source': 'straight', 'dest': 'pause_straight'},
            {'trigger': 'pause', 'source': 'arc', 'dest': 'pause_arc'},
            {'trigger': 'resume', 'source': 'pause_straight', 'dest': 'straight'},
            {'trigger': 'resume', 'source': 'pause_arc', 'dest': 'arc'}
        ]
        machine = GraphMachine(model=self._driving_machine, states=states, transitions=transitions, initial='stop')
    
    def print_machine(self, info):
        """Print the current state of these machines."""
        if self._debug_machine:
            self._driving_machine.get_graph().draw(f'driving_machine_{self.print_count}_{info}.png', prog='dot')
            self._autonomy_machine.get_graph().draw(f'autonomy_machine_{self.print_count}_{info}.png', prog='dot')
            self.print_count += 1

    # Service callbacks ---------------------------------------------------------------------------
    def enable_twist(self):
        ack_request = Enable.Request()
        ack_request.enable = True
        self._client_futures.append(self.enable_twist_client.call_async(ack_request))

    def disable_twist(self):
        ack_request = Enable.Request()
        ack_request.enable = False
        self._client_futures.append(self.enable_twist_client.call_async(ack_request))

    # Subscriber callbacks ------------------------------------------------------------------------
    def command_callback(self, msg):
        """Responding to a received command."""
        # self.command_pub.publish(msg)
        payload = msg.operational_mode.lower()
        self.get_logger().debug(f"RECEIVED MESSAGE: {payload}")
        try:
            if str(self._autonomy_machine.state).lower() != payload:
                self._autonomy_machine.trigger(payload)
            else:
                self.get_logger().warning(f"Already in {payload} mode!")
        except AttributeError:
            self.get_logger().warning(f"Received {payload}: NOT a valid mode!")

    def figure_eight_experiment(self):
        """Switch driving angle between both extremes on alternating calls.
        For the best figure-8, use a callback period of 2.7s.
        """
        self._timer_counter += 1
        if self._timer_counter % 2 == 0:
            self._driving_machine.stop()
            self.update_arc_params(1.2, 25.0)
            self._driving_machine.arc()
        else:
            self._driving_machine.stop()
            self.update_arc_params(1.2, -25.0)
            self._driving_machine.arc()

    def straight_experiment(self):
        """Start and stop the vehicle on alternating calls."""
        self._timer_counter += 1
        if self._timer_counter % 2 == 0:
            self._driving_machine.straight()
        else:
            self._driving_machine.stop()

    def update_straight_params(self, velocity):
        """Update the velocity parameter of the straight_behavior node by calling its parameter-
        setting service.
        """
        parameter = Parameter()
        parameter.name = 'velocity'
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = velocity
        self.behavior_param_request.parameters.append(parameter)

        self._client_futures.append(self.straight_param_client.call_async(self.behavior_param_request))
    def update_arc_velocity(self, velocity):
        """Update the velocity parameter of the arc_behavior node by calling its parameter-
        setting service.
        """
        parameter = Parameter()
        parameter.name = 'velocity'
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = velocity
        self.behavior_param_request.parameters.append(parameter)

        self._client_futures.append(self.arc_param_client.call_async(self.behavior_param_request))

    def update_arc_steering_angle(self, velocity):
        """Update the steering_angle parameter of the arc_behavior node by calling its parameter-
        setting service.
        """
        parameter = Parameter()
        parameter.name = 'steering_angle'
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = velocity
        self.behavior_param_request.parameters.append(parameter)

        self._client_futures.append(self.arc_param_client.call_async(self.behavior_param_request))

    def update_arc_params(self, velocity, steering_angle):
        """Update the velocity AND steering_angle parameters of the arc_behavior node by calling
        those respective methods.
        """
        self.update_arc_velocity(velocity)
        self.update_arc_steering_angle(steering_angle)
    
    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures=[]
            for f in self._client_futures:
                if f.done():
                    self.get_logger().info(f"Service result: {f.result()}")
                else:
                    self.get_logger().warn(f"FUTURE INCOMPLETE!")
                    incomplete_futures.append(f)
            self._client_futures = incomplete_futures


class AutonomyMachine:
    """Callbacks for autonomy mode state transitions."""
    def __init__(self, autonomy_manager):
        self._autonomy_manager = autonomy_manager
        self._agent_name = self._autonomy_manager._agent_name

    # Direct mode   -----------------------------
    def on_enter_direct(self):
        self._autonomy_manager.get_logger().info(f"AutonomyMachine is now entering DIRECT mode.")
        self._autonomy_manager.enable_twist()
        self._autonomy_manager.print_machine("enter_direct")

    def on_exit_direct(self):
        self._autonomy_manager.get_logger().info(f"AutonomyMachine is now exiting DIRECT mode.")
        self._autonomy_manager.disable_twist()
        self._autonomy_manager.print_machine("exit_direct")

    # Auto mode     -----------------------------
    def on_enter_auto(self):
        self._autonomy_manager.get_logger().info(f"AutonomyMachine is now entering AUTO mode.")
        self._autonomy_manager.print_machine("enter_auto")

    def on_exit_auto(self):
        self._autonomy_manager.get_logger().info(f"AutonomyMachine is now exiting AUTO mode.")
        self._autonomy_manager.print_machine("exit_auto")

    # Experiment mode   -------------------------
    def on_enter_experiment(self):
        self._autonomy_manager.get_logger().info(f"AutonomyMachine is now entering EXPERIMENT mode.")
        self._autonomy_manager.mqtt_client.subscribe(self._autonomy_manager.demo_topic)
        # self._autonomy_manager.update_straight_params(0.5)
        self._autonomy_manager._driving_machine.straight()
        self._autonomy_manager.print_machine("enter_experiment")
        # self._autonomy_manager.timer = self._autonomy_manager.create_timer(
        #     2.7, self._autonomy_manager.figure_eight_experiment
        # )

    def on_exit_experiment(self):
        self._autonomy_manager.get_logger().info(f"AutonomyMachine is now exiting EXPERIMENT mode.")
        self._autonomy_manager.mqtt_client.unsubscribe(self._autonomy_manager.demo_topic)
        # self._autonomy_manager.timer.cancel()
        self._autonomy_manager._driving_machine.stop()
        self._autonomy_manager.print_machine("exit_experiment")


class DrivingMachine:
    """Callbacks for driving mode state transitions."""
    def __init__(self, autonomy_manager):
        self._autonomy_manager = autonomy_manager
        self._agent_name = self._autonomy_manager._agent_name

    # STOP  -------------------------------------
    def on_enter_stop(self):
        self._autonomy_manager.get_logger().info(f"DrivingMachine is now entering STOP mode.")

    def on_exit_stop(self):
        self._autonomy_manager.get_logger().info(f"DrivingMachine is now exiting STOP mode.")

    # STRAIGHT  ---------------------------------
    def on_enter_straight(self):
        self._autonomy_manager.get_logger().info(f"DrivingMachine is now entering STRAIGHT mode.")
        self._autonomy_manager.behavior_request.enable = True
        self._autonomy_manager._client_futures.append(self._autonomy_manager.straight_behavior_client.call_async(
            self._autonomy_manager.behavior_request))

    def on_exit_straight(self):
        self._autonomy_manager.get_logger().info(f"DrivingMachine is now exiting STRAIGHT mode.")
        self._autonomy_manager.behavior_request.enable = False
        self._autonomy_manager._client_futures.append(self._autonomy_manager.straight_behavior_client.call_async(
            self._autonomy_manager.behavior_request))

    # ARC   -------------------------------------
    def on_enter_arc(self):
        self._autonomy_manager.get_logger().info(f"DrivingMachine is now entering ARC mode.")
        self._autonomy_manager.behavior_request.enable = True
        self._autonomy_manager._client_futures.append(self._autonomy_manager.arc_behavior_client.call_async(
            self._autonomy_manager.behavior_request))

    def on_exit_arc(self):
        self._autonomy_manager.get_logger().info(f"DrivingMachine is now exiting ARC mode.")
        self._autonomy_manager.behavior_request.enable = False
        self._autonomy_manager._client_futures.append(self._autonomy_manager.arc_behavior_client.call_async(
            self._autonomy_manager.behavior_request))

def main():
    """Boilerplate ROS node spinup."""
    rclpy.init()
    node = AutonomyManager()
    # rclpy.spin(node)
    node.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
