import rclpy
from rclpy.node import Node

from traffic_signal_msgs.msg import LanternState
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

    def _init_pub(self):
        self._car_state_pub = self.create_publisher(
            msg_type=PoseStamped,
            topic=self._output_topic,
            qos_profile=10,
        )

    def _init_sub(self):
        """No sensing data being collected / parsed as of yet."""
        pass

    def _init_dict(self):
        self._agent_states = {}
        init_status = CarInfo(0, 0)
        self._agent_states[self.get_namespace()] = init_status

    def _timer_cb(self):
        state_msg = PoseStamped()
        this_car_status = self._agent_states[self.get_namespace()]

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