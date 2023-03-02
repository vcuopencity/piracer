# Standard library imports
import json
from os import environ

# 3rd-party imports
from carma_v2x_msgs.msg import BSM
from nav_msgs.msg import Odometry
from opencity_utils import state_msgs
from rclpy.node import Node
from rosidl_runtime_py import convert
from rosidl_runtime_py import set_message
from traffic_signal_msgs.msg import SignalState

import paho.mqtt.client
import rclpy


class V2xNode(Node):
    """Report state of car and receive states of other agents over MQTT. All states are stored in
    _agent_states in their respective agent information types, currently implemented: car_state
    and signal_state.
    """

    def __init__(self):
        super().__init__('v2x_node')

        self._agent_name = self.get_namespace().lstrip('/')

        self._init_params()
        self._init_mqtt()
        self._init_sub()
        self._init_dict()

        self._timer = self.create_timer(self._report_freq, callback=self._timer_cb)

    def _init_params(self):
        self.declare_parameter('car_state_output_topic', 'car_state_output_topic')
        self._output_topic = self.get_parameter('car_state_output_topic').get_parameter_value().string_value

        self.declare_parameter('report_frequency', 1.0)
        self._report_freq = self.get_parameter('report_frequency').get_parameter_value().double_value

        self.declare_parameter('neighbor_list', [])
        self._neighbor_list = self.get_parameter('neighbor_list').get_parameter_value().string_array_value

        self.declare_parameter('location_input')
        self._location_input = self.get_parameter('location_input').get_parameter_value().string_value

        if self._location_input == '':
            raise ValueError("'location_input' parameter not set in config file.")

    def _init_sub(self):
        """Subscribe to all MQTT neighbor state topics and the ROS2 location topic."""
        for neighbor in self._neighbor_list:
            if self._agent_name in neighbor:
                continue
            elif "car" in neighbor:
                self.mqtt_client.subscribe(f'/{neighbor}/car_state')
            elif "signal" in neighbor:
                self.mqtt_client.subscribe(f'/{neighbor}/signal_state')
        
        self._location_sub = self.create_subscription(
            msg_type=Odometry,
            topic=self._location_input,
            callback=self.location_cb,
            qos_profile=10,
        )

    def _init_mqtt(self):
        """Create an MQTT client, set callbacks, connect to the broker, and start the loop."""
        self.mqtt_client = paho.mqtt.client.Client(f'/{self._agent_name}/v2x_node')

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
            self.get_logger().fatal(f'MQTT Publisher failed to connect to: {self._mqtt_broker_uri} Node is terminating...')
            exit()
        

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f'MQTT Publisher connected to: {self._mqtt_broker_uri} with result code: {rc}')

    def _mqtt_on_message(self, client, userdata, msg):
        """Determine the state type that was received and forward it to the appropriate function for storing."""
        topic_info = msg.topic.split('/')
        state_type = topic_info[-1]
        agent_name = topic_info[-2]

        if state_type == 'car_state':
            self._update_car_info(msg, agent_name)
        elif state_type == 'signal_state':
            self._update_signal_info(msg, agent_name)

    def _update_car_info(self, mqtt_msg, agent_name):
        """Convert MQTT msg to dict, then to ROS message. Store this information in _agent_states under agent_name."""
        dict_msg = json.loads(mqtt_msg.payload.decode('utf-8'))
        ros_msg = BSM()
        set_message.set_message_fields(ros_msg, dict_msg)
        new_info = state_msgs.CarInfo(latitude=ros_msg.core_data.latitude, 
                                      longitude=ros_msg.core_data.longitude)
        self._agent_states[agent_name] = new_info

    def _update_signal_info(self, mqtt_msg, agent_name):
        """Convert MQTT msg to dict, then to ROS message. Store this information in _agent_states under agent_name."""
        dict_msg = json.loads(mqtt_msg.payload.decode('utf-8'))
        ros_msg = SignalState()
        set_message.set_message_fields(ros_msg, dict_msg)
        new_info = state_msgs.SignalInfo(states=ros_msg.states.states, 
                                         x=ros_msg.pose.pose.position.x,
                                         y=ros_msg.pose.pose.position.y)
        self._agent_states[agent_name] = new_info

    def _init_dict(self):
        self._agent_states = {}
        self._set_state(state_msgs.CarInfo(latitude=0, longitude=0))
    
    def _set_state(self, new_info: state_msgs.CarInfo):
        """Set this agent's state."""
        self._agent_states[self._agent_name] = new_info
    
    def _get_state(self) -> state_msgs.CarInfo:
        """Return this agent's state."""
        return self._agent_states[self._agent_name]

    def location_cb(self, msg):
        """Update this agent's state with received Odom msg."""
        self.get_logger().debug(f"Received x:{msg.pose.pose.position.x} y:{msg.pose.pose.position.y}")
        self._set_state(state_msgs.CarInfo(latitude=msg.pose.pose.position.x,
                                           longitude=msg.pose.pose.position.y))

    def _timer_cb(self):
        """Create ROS message containing this agent's state from _agent_states, convert it to a dictionary,
        convert it to a byte type, and then publish it via MQTT."""
        state_msg = BSM()
        this_car_status = self._get_state()

        state_msg.core_data.latitude = float(this_car_status.latitude)
        state_msg.core_data.longitude = float(this_car_status.longitude)

        state_msg_dict = convert.message_to_ordereddict(state_msg)
        state_msg_byte = json.dumps(state_msg_dict).encode('utf-8')
        self.mqtt_client.publish(f'/{self._agent_name}/{self._output_topic}', state_msg_byte)

        self.get_logger().debug(str(self._agent_states))


def main():
    """Boilerplate ROS node spin-up."""
    rclpy.init()
    node = V2xNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
