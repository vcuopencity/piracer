# Third-party imports
from autonomy_mgmt_msgs.srv import Enable
from geometry_msgs.msg import Twist
from rclpy.node import Node

import rclpy


class ArcBehavior(Node):
    """Drive the vehicle forward at the velocity determined by the parameter when enabled."""

    def __init__(self):
        super().__init__('arc_behavior')

        self._agent_name = self.get_namespace().lstrip('/')

        self._init_params()
        self._init_srv()
        self._init_pub()

    def _init_params(self):
        self.declare_parameter('velocity', 1.0)
        # Velocity not set to a member variable here because it is meant to be changed after initialization,
        # use self._get_velocity().
        self.declare_parameter('steering_angle', 45.0)
        # Steering angle not set to a member variable here because it is meant to be changed after initialization,
        # use self._get_steering_angle().

        self.declare_parameter('twist_topic', 'twist_topic')
        self._twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value

    def _init_srv(self):
        self._srv = self.create_service(Enable, 'arc_behavior_enable_service', self._enable_srv)

    def _init_pub(self):
        self._twist_pub = self.create_publisher(
            msg_type=Twist,
            topic=self._twist_topic,
            qos_profile=10,
        )

    def _enable_srv(self, request, response):
        twist_msg = Twist()

        if request.enable:
            twist_msg.linear.x = self._get_velocity()
            twist_msg.angular.z = self._get_steering_angle()
            self._twist_pub.publish(twist_msg)
            response.result = True
            response.status = f"{self._agent_name} is now moving at {self._get_velocity()} m/s at " \
                              f"{self._get_steering_angle()} degrees."
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self._twist_pub.publish(twist_msg)
            response.result = True
            response.status = f"{self._agent_name} is now stopped."

        return response

    def _get_velocity(self):
        return self.get_parameter('velocity').get_parameter_value().double_value

    def _get_steering_angle(self):
        return self.get_parameter('steering_angle').get_parameter_value().double_value


def main():
    """Boilerplate ROS node spin-up."""
    rclpy.init()
    node = ArcBehavior()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
