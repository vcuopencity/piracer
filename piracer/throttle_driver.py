import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import FloatingPointRange

from piracer_msgs.msg import Throttle

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor.motor import DCMotor


class ThrottleDriver(Node):
    """Sub to /agent/throttle and output throttle commands to PCA9685 DC Motor via I2C while
    ensuring the commands are within min_throttle and max_throttle constraints."""
    def __init__(self):
        super(ThrottleDriver, self).__init__('throttle_driver')

        fp_range = FloatingPointRange(from_value=-1.0, to_value=1.0, step=0.0)
        desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                   description='Minimum throttle value',
                                   floating_point_range=[fp_range])
        self.min_throttle = self.declare_parameter('min_throttle', value=-1.0,
                                                   descriptor=desc)

        desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                   description='Maximum throttle value',
                                   floating_point_range=[fp_range])
        self.max_throttle = self.declare_parameter('max_throttle', value=1.0,
                                                   descriptor=desc)

        self._subscriber = self.create_subscription(msg_type=Throttle,
                                                    topic='throttle',
                                                    callback=self._msg_cb,
                                                    qos_profile=10)

        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x60)
        pca.frequency = 60.0

        self.motor = DCMotor(pca.channels[3], pca.channels[4])
        self.motor.throttle = 0.0

    def _msg_cb(self, msg: Throttle):
        self.motor.throttle = min(self.get_parameter('max_throttle').value,
                                  max(self.get_parameter('min_throttle').value,
                                      msg.percent))


def main():
    rclpy.init()
    node = ThrottleDriver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
