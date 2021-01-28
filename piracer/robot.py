import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Image

import adafruit_motorkit
import adafruit_servokit

import cv2
import numpy as np


_CURRENT_SENSE_ADDRESS = 0x41
_OLED_ADDRESS = 0x3c
_PCA9685_SERVO_ADDRESS = 0x40
_PCA9685_MOTOR_ADDRESS = 0x60


_QUEUE_DEPTH = 10


class Robot(Node):
    def __init__(self):
        super(Robot, self).__init__('robot')

        # Servo control
        servo_kit = adafruit_servokit.ServoKit(channels=16, address=_PCA9685_SERVO_ADDRESS)
        self._steering_servo = servo_kit.servo[0]

        # Motor control
        motor_kit = adafruit_motorkit.MotorKit(address=_PCA9685_MOTOR_ADDRESS)
        self._accelerator = motor_kit.motor3

        # Camera
        self.camera = cv2.VideoCapture(0)

        # Subscribers
        self.message_subscriber = self.create_subscription(msg_type=AckermannDrive,
                                                           topic='cmd',
                                                           callback=self.message_callback,
                                                           qos_profile=_QUEUE_DEPTH)

        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        profile = QoSProfile(history=1,
                             depth=1,
                             reliability=2,
                             durability=2)

        # Publishers
        self.image_publisher = self.create_publisher(msg_type=Image,
                                                     topic='camera',
                                                     qos_profile=profile)

        # Timers
        self.image_timer = self.create_timer(timer_period_sec=1,
                                             callback=self.image_timer_callback)

    def message_callback(self, msg: AckermannDrive):
        self._steering_servo.angle = int(msg.steering_angle)
        self._accelerator.throttle = msg.speed

        print(f'angle: {self._steering_servo.angle}  speed: {self._accelerator.throttle}')

    def image_timer_callback(self):
        success, frame = self.camera.read()

        if not success:
            self.get_logger().error('Could not read from camera')

        msg = Image()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = '/img_frame'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.step = 640

        image_data = frame.astype(np.uint8)
        msg.data = image_data.flatten().tolist()

        self.image_publisher.publish(msg)

        self.get_logger().info('Published image')


def main():
    rclpy.init()
    platform = Robot()
    rclpy.spin(platform)

    platform.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
