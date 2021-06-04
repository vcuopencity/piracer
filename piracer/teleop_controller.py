import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class TeleopController(Node):
    def __init__(self):
        super(TeleopController, self).__init__('teleop_controller')

        self.create_subscription(msg_type=Joy,
                                 topic='joy',
                                 callback=self._msg_cb,
                                 qos_profile=10)

        self.throttle_pub = self.create_publisher(msg_type=Float64,
                                                  topic='throttle',
                                                  qos_profile=10)

        self.angle_pub = self.create_publisher(msg_type=Float64,
                                               topic='angle',
                                               qos_profile=10)

    def _msg_cb(self, msg: Joy):
        throttle_msg = Float64()
        throttle_msg.data = msg.axes[1]
        self.throttle_pub.publish(throttle_msg)

        angle_msg = Float64()
        angle_msg.data = - msg.axes[3] * 90
        self.angle_pub.publish(angle_msg)


def main():
    rclpy.init()
    node = TeleopController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
