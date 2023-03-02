import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import FloatingPointRange



class ViveDriver(Node):
    """Subscribe to VIVE tracker Odom messages and publish them to V2X node."""
    def __init__(self):
        super().__init__('vive_driver')
        self._init_params()
        self._init_pub()
        self._init_sub()


    def _init_params(self):
        """Initialize and validate parameters."""
        self.declare_parameter('input_topic')
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        if self.input_topic == '':
            raise ValueError("'input_topic' parameter not set in config file.")
        
        self.declare_parameter('output_topic')
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        if self.output_topic == '':
            raise ValueError("'output_topic' parameter not set in config file.")
    
    def _init_pub(self):
        """Init internal publisher."""
        self._pub = self.create_publisher(
            msg_type=Odometry,
            topic=self.output_topic,
            qos_profile=10,
        )
    
    def _init_sub(self):
        """Init VIVE subscriber."""
        self._sub = self.create_subscription(
            msg_type=Odometry,
            topic=self.input_topic,
            callback=self._msg_cb,
            qos_profile=10,
        )

    def _msg_cb(self, msg: Odometry):
        """Re-publish received message internally."""
        self._pub.publish(msg)

def main():
    rclpy.init()
    node = ViveDriver()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
