import rclpy
from rclpy.node import Node

from traffic_signal_msgs.msg import SignalState
from geometry_msgs.msg import PoseStamped


class CarInfo:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class CarState(Node):
    """Report state of car and receive states of other agents. All states are stored in _agent_states in their
    respective agent information types. Currently, only CarInfo is implemented.
    """

    def __init__(self):
        super().__init__('car_state')

        self._agent_name = self.get_namespace().lstrip('/')

        self._init_params()
        self._init_pub()
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

    def _init_pub(self):
        self._car_state_pub = self.create_publisher(
            msg_type=PoseStamped,
            topic=self._output_topic,
            qos_profile=10,
        )

    def _init_sub(self):
        """Subscribe to all neighbor state topics."""
        for neighbor in self._neighbor_list:
            if self._agent_name in neighbor:
                return
            elif "car" in neighbor:
                self.create_subscription(
                    msg_type=PoseStamped,
                    topic='/' + neighbor + '/car_state',
                    callback=self._neighbor_cb,
                    qos_profile=10,
                )
            elif "signal" in neighbor:
                self.create_subscription(
                    msg_type=SignalState,
                    topic='/' + neighbor + '/signal_state',
                    callback=self._neighbor_cb,
                    qos_profile=10,
                )

    def _init_dict(self):
        self._agent_states = {}
        init_status = CarInfo(0, 0)
        self._agent_states[self._agent_name] = init_status

    def _neighbor_cb(self, msg):
        pass

    def _timer_cb(self):
        state_msg = PoseStamped()
        this_car_status = self._agent_states[self._agent_name]

        state_msg.pose.position.x = float(this_car_status.x)
        state_msg.pose.position.y = float(this_car_status.y)

        self._car_state_pub.publish(state_msg)


def main():
    """Boilerplate ROS node spin-up."""
    rclpy.init()
    node = CarState()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
