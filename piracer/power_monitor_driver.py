import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

import board
import busio
from adafruit_ina219 import INA219


class PowerMonitorDriver(Node):
    def __init__(self):
        super(PowerMonitorDriver, self).__init__('power_monitor_driver')

        i2c = busio.I2C(board.SCL, board.SDA)
        self._ina = INA219(i2c, addr=0x41)

        self.power_pub = self.create_publisher(msg_type=Float64,
                                               topic='current',
                                               qos_profile=10)

        self.create_timer(timer_period_sec=1, callback=self._timer_cb)

    def _timer_cb(self):
        msg = Float64()
        msg.data = self._ina.current

        self.power_pub.publish(msg)


def _main():
    rclpy.init()
    node = PowerMonitorDriver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    _main()
