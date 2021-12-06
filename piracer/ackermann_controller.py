import rclpy
import numpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

from piracer_msgs.msg import SteeringAngle, Throttle


class AckermannController(Node):
    """Create a pub/sub for the ackermann drive bridge in the mqtt_bridge package, create publishers for the hardware
    nodes, create a service server for the autonomy_manager to control whether or not ackermann control is active.
    """

    def __init__(self):
        super().__init__('ackermann_driver')

        self._agent_name = self.get_namespace().lstrip('/')

        self._init_params()
        self._init_sub()
        self._init_pub()
        self._init_srv()

    def _init_params(self):
        self.declare_parameter('twist_bridge_input_topic', 'ros_pub_top')
        self.twist_input_topic = self.get_parameter('twist_bridge_input_topic').get_parameter_value().string_value

        self.declare_parameter('twist_bridge_output_topic', 'ros_sub_top')
        self.twist_output_topic = self.get_parameter('twist_bridge_output_topic').get_parameter_value().string_value

        self.declare_parameter('vehicle_length', 1.0)
        self.vehicle_length = self.get_parameter('vehicle_length').get_parameter_value().double_value

        self.declare_parameter('max_velocity', 2.5)
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        self.declare_parameter('min_velocity', 0.001)
        self.min_velocity = self.get_parameter('min_velocity').get_parameter_value().double_value

        self.declare_parameter('min_omega', 0.001)
        self.min_omega = self.get_parameter('min_omega').get_parameter_value().double_value

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

    def _init_srv(self):
        service_name = f'/{self._agent_name}/steering_driver/get_parameters'
        self.get_param_client = self.create_client(GetParameters, service_name)
        while not self.get_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"{service_name} not available, waiting...")
        request = GetParameters.Request()
        request.names = ['min_angle', 'max_angle']

        future = self.get_param_client.call_async(request)
        future.add_done_callback(self.get_param_callback)

    def _twist_callback(self, msg):
        """Calculate a new steering angle and throttle amount based on incoming twist message and
        publish them to the steering and throttle nodes.
        """
        velocity = msg.linear.x
        omega = msg.angular.z

        if abs(velocity) < self.min_velocity or abs(omega) < self.min_omega:
            velocity = 0.0
            phi = 0.0
        else:
            phi = numpy.arctan((self.vehicle_length * omega) / velocity)

        throttle_msg = Throttle()
        throttle_msg.percent = self._parse_velocity(velocity)

        steer_msg = SteeringAngle()
        steer_msg.radian = phi

        self.throttle_pub.publish(throttle_msg)
        self.steering_pub.publish(steer_msg)

        self.twist_pub.publish(msg)

    def _parse_velocity(self, velocity):
        throttle_amount = velocity / self.max_velocity
        return throttle_amount

    def get_param_callback(self, future):
        result = future.result()
        self._min_angle = result.values[0].double_value
        self._max_angle = result.values[1].double_value
        self.get_logger().debug(f"Min angle: {self._min_angle} Max angle: {self._max_angle}")


def main():
    """Boilerplate ROS node spinup."""
    rclpy.init()
    node = AckermannController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
