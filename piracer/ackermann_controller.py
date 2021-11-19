import rclpy
import numpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


from piracer_msgs.msg import SteeringAngle, Throttle


class AckermannController(Node):
    """Create a pub/sub for the ackermann drive bridge in the mqtt_bridge package, create publishers for the hardware
    nodes, create a service server for the autonomy_manager to control whether or not ackermann control is active.
    """
    def __init__(self):
        super().__init__('ackermann_driver')

        self._init_params()
        self._init_sub()
        self._init_pub()

    def _init_params(self):

        self.declare_parameter('twist_bridge_input_topic', 'ros_pub_top')
        self.twist_input_topic = self.get_parameter('twist_bridge_input_topic').get_parameter_value().string_value

        self.declare_parameter('twist_bridge_output_topic', 'ros_sub_top')
        self.twist_output_topic = self.get_parameter('twist_bridge_output_topic').get_parameter_value().string_value

        self.declare_parameter('vehicle_length', 1.0)
        self.vehicle_length = self.get_parameter('vehicle_length').get_parameter_value().double_value

        self.declare_parameter('maximum_velocity', 2.5)
        self.max_velocity = self.get_parameter('maximum_velocity').get_parameter_value().double_value

    def _init_sub(self):
        self._twist_sub = self.create_subscription(
            msg_type=Twist,
            topic=self.twist_input_topic,
            callback=self._twist_callback,
            qos_profile=10,
        )

    def _init_pub(self):
        self.twist_pub = self.create_publisher(
            msg_type=Twist,
            topic=self.twist_output_topic,
            qos_profile=10,
        )
        # Publishers for hardware nodes
        self.steering_pub = self.create_publisher(
            msg_type=SteeringAngle,
            topic='angle',
            qos_profile=10,
        )
        self.throttle_pub = self.create_publisher(
            msg_type=Throttle,
            topic='throttle',
            qos_profile=10,
        )

    def _twist_callback(self, msg):
        """Calculate a new steering angle and throttle amount based on incoming twist message and
        publish them to the steering and throttle nodes.
        """
        velocity = msg.linear.x
        omega = msg.angular.z

        if abs(velocity) < 0.001:
            velocity = 0.0
            phi = 0.0
        else:
            phi = numpy.arctan((self.vehicle_length*omega)/velocity)

        throttle_msg = Throttle()
        throttle_msg.percent = self._parse_velocity(velocity)

        steer_msg = SteeringAngle()
        steer_msg.radian = phi

        self.throttle_pub.publish(throttle_msg)
        self.steering_pub.publish(steer_msg)

        self.twist_pub.publish(msg)

    def _parse_velocity(self, velocity):
        throttle_amount = velocity/self.max_velocity
        return throttle_amount


def main():
    """Boilerplate ROS node spinup."""
    rclpy.init()
    node = AckermannController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
