import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

from piracer_msgs.msg import SteeringAngle

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor.servo import Servo


class SteeringDriver(Node):
    def __init__(self):
        super(SteeringDriver, self).__init__('steering_driver')

        desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                   description='Minimum steering angle')
        self._min_angle = self.declare_parameter('min_angle', value=0.0,
                                                 descriptor=desc)

        desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                   description='Maximum steering angle')
        self._max_angle = self.declare_parameter('max_angle', value=0.0,
                                                 descriptor=desc)

        self._subscriber = self.create_subscription(msg_type=SteeringAngle,
                                                    topic='angle',
                                                    callback=self._msg_cb,
                                                    qos_profile=10)

        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 50.0

        self.servo = Servo(pca.channels[0])
        self.servo.angle = 90.0

    def _msg_cb(self, msg: SteeringAngle):
        angle = min(self.get_parameter('max_angle').value,
                    max(self.get_parameter('min_angle').value,
                        msg.radian))
        self.servo.angle = angle + 90


def main():
    rclpy.init()
    node = SteeringDriver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
