# Third-party imports
from autonomy_mgmt_msgs.srv import Enable
from cmd_msgs.msg import Command
from transitions import Machine
from rclpy.node import Node

import rclpy


class StraightBehavior(Node):
    """Drive the vehicle forward at the velocity determined by the parameter when enabled."""

    def __init__(self):
        super().__init__('straight_behavior')

    def _init_params(self):
        self.declare_parameter('velocity', 1.0)
        self._velocity = self.get_parameter('velocity').get_parameter_value().double_value

    def _init_srv(self):
        pass


def main():
    """Boilerplate ROS node spin-up."""
    rclpy.init()
    node = StraightBehavior()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


