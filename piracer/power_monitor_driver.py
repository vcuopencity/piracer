import rclpy
from rclpy.node import Node

from piracer_msgs.msg import SystemPower

import board
import busio
from adafruit_ina219 import INA219


class PowerMonitorDriver(Node):
    """Pub SystemPower messages from INA219 DC current sensor over I2C to /agent/current."""
    def __init__(self):
        super(PowerMonitorDriver, self).__init__('power_monitor_driver')

        i2c = busio.I2C(board.SCL, board.SDA)
        self._ina = INA219(i2c, addr=0x41)

        self.power_pub = self.create_publisher(msg_type=SystemPower,
                                               topic='current',
                                               qos_profile=10)

        self.create_timer(timer_period_sec=1, callback=self._timer_cb)

    def _timer_cb(self):
        msg = SystemPower()

        msg.voltage = self._ina.bus_voltage + self._ina.shunt_voltage
        msg.current = self._ina.current
        msg.power = self._ina.power

        self.power_pub.publish(msg)


def main():
    rclpy.init()
    node = PowerMonitorDriver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
