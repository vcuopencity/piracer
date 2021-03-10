import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import FloatingPointRange

from std_msgs.msg import Float64

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import Servo


class SteeringDriver(Node):
    def __init__(self):
        super(SteeringDriver, self).__init__('steering_driver')

        fp_range = FloatingPointRange(from_value=-45.0,
                                      to_value=45.0,
                                      step=0.0)
        desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                   description='Minimum steering angle',
                                   floating_point_range=[fp_range])
        self._min_angle = self.declare_parameter('min_angle', value=-45.0,
                                                 descriptor=desc)

        desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                   description='Maximum steering angle',
                                   floating_point_range=[fp_range])
        self._max_angle = self.declare_parameter('max_angle', value=45.0,
                                                 descriptor=desc)

        self._subscriber = self.create_subscription(msg_type=Float64,
                                                    topic='angle',
                                                    callback=self._msg_cb,
                                                    qos_profile=10)

        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 50.0

        self.servo = Servo(pca.channels[0])
        self.servo.angle = 90.0

    def _msg_cb(self, msg: Float64):
        self.servo.angle = msg.data


def main():
    rclpy.init()
    node = SteeringDriver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
