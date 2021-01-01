import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive

import adafruit_motorkit
import adafruit_servokit


class Platform(Node):
    def __init__(self):
        super(Platform, self).__init__('platform')
        self.msg_sub = self.create_subscription(AckermannDrive, 'control', self.msg_callback, 10)

        servo_kit = adafruit_servokit.ServoKit(channels=16, address=0x40)
        self._steering_servo = servo_kit.servo[0]

        motor_kit = adafruit_motorkit.MotorKit()
        self._accelerator = motor_kit.motor3

    def msg_callback(self, msg: AckermannDrive):
        self._steering_servo.angle = int(msg.steering_angle)
        self._accelerator.throttle = msg.speed

        print(f'angle: {self._steering_servo.angle}  speed: {self._accelerator.throttle}')


def main():
    rclpy.init()
    platform = Platform()
    rclpy.spin(platform)

    platform.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
