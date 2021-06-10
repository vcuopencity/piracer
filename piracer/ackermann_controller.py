import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64


class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')

        #  initialzing the necessary subscriber and publishers
        self.ackermann_sub = self.create_subscription(
            msg_type = AckermannDrive, 
            topic = '/ros_pub_top', 
            callback = self.ackermann_callback,
            qos_profile = 10,
        )

        self.steering_pub = self.create_publisher(
            msg_type = Float64,
            topic = 'angle',
            qos_profile = 10,
        )
        self.throttle_pub = self.create_publisher(
            msg_type = Float64,
            topic = 'throttle',
            qos_profile = 10,
        )

    # What to do when a message is recieved
    def ackermann_callback(self, msg):
        self.get_logger().debug('RECEIVED | angle: %f angle_vel: %f speed: %f accel: %f jerk: %f' 
        % (msg.steering_angle, msg.steering_angle_velocity, msg.speed, msg.acceleration, msg.jerk))
        
        throttle_msg = Float64()
        throttle_msg.data = msg.speed

        steer_msg = Float64()
        steer_msg.data = msg.steering_angle

        self.throttle_pub.publish(throttle_msg)         # Publishing the 
        self.steering_pub.publish(steer_msg)            # actual messages


def main():
    rclpy.init()
    node = AckermannDriver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
