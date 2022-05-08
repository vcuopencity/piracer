import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from piracer_msgs.msg import SteeringAngle, Throttle


class TeleopController(Node):
    """Sub to /agent/joy and output /agent/throttle and /agent/angle commands."""
    def __init__(self):
        super(TeleopController, self).__init__('teleop_controller')

        self.create_subscription(msg_type=Joy,
                                 topic='joy',
                                 callback=self._msg_cb,
                                 qos_profile=10)

        self.throttle_pub = self.create_publisher(msg_type=Throttle,
                                                  topic='throttle',
                                                  qos_profile=10)

        self.angle_pub = self.create_publisher(msg_type=SteeringAngle,
                                               topic='angle',
                                               qos_profile=10)

    def _msg_cb(self, msg: Joy):
        throttle_msg = Throttle()
        throttle_msg.percent = msg.axes[1]
        self.throttle_pub.publish(throttle_msg)

        angle_msg = SteeringAngle()
        angle_msg.radian = - msg.axes[3] * 1.309
        self.angle_pub.publish(angle_msg)


def main():
    rclpy.init()
    node = TeleopController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
