import subprocess

import rclpy
from rclpy.node import Node

import board
import busio
from adafruit_ssd1306 import SSD1306_I2C

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


class DisplayDriver(Node):
    """Output IP address and ROS2 namespace to SSD1306_I2C display over I2C."""
    def __init__(self):
        super(DisplayDriver, self).__init__('display_driver')

        i2c = busio.I2C(board.SCL, board.SDA)
        self.display = SSD1306_I2C(128, 32, i2c)

        self.display.fill(0)
        self.display.show()

        width = self.display.width
        height = self.display.height
        image = Image.new("1", (width, height))
        self.draw = ImageDraw.Draw(image)

        self.font = ImageFont.load_default()

        cmd = "hostname -I | cut -d' ' -f1"
        ip = subprocess.check_output(cmd, shell=True).decode('utf-8')

        padding = -2
        top = padding
        bottom = height - padding

        self.draw.rectangle((0, 0, width, height), outline=0, fill=0)
        self.draw.text((0, top + 0), 'Name: ' + self.get_namespace(), font=self.font, fill=255)
        self.draw.text((0, top + 8), 'IP: ' + ip, font=self.font, fill=255)

        self.display.image(image)
        self.display.show()


def main():
    rclpy.init()
    node = DisplayDriver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
