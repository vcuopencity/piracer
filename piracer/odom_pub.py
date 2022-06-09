import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from os import environ


class PublishOdom(Node):
    def __init__(self):
        super().__init__('odom_pub')

        # Declaring parameters
        self.declare_parameter('imu_topic', 'imu')
        self.car_id = environ['CAR_ID']
        self.tracker_id = environ['TRACKER_ID']

        # Reading in parameters for node, by default the parameters can be found
        # under config/params.yaml
        self.vive_topic = "/vive_odom/" + self.tracker_id
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
       
        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)

        # Create topic subscriber for IMU and VIVE
        self.imu_input = self.create_subscription(Imu, self.imu_topic, self._process_imu_msg, 1)
        self.vive_input = self.create_subscription(Odometry, self.vive_topic, self._process_vive_msg, 1)
        
        # Create topic publisher for car odometry publisher
        self.odom_output = self.create_publisher(Odometry, 'odom', 1)

        # Create timer for odometry publisher
        self.rate = 60  # 60 Hz
        self.timer_period = 1.0 / self.rate
        self.timer = self.create_timer(self.timer_period, self._publish_odom_msg)

        # Declaring variable that stores current IMU and Vive tracker location readings
        self.imu = Imu()
        self.vive = Odometry()

    def _publish_odom_msg(self):

        odomMsg = Odometry()
        odomMsg.header.stamp = self.get_clock().now().to_msg()
        odomMsg.header.frame_id = self.car_id + '_base_link'

        odomMsg.pose.pose.position.x = float(self.vive.pose.pose.position.x)
        odomMsg.pose.pose.position.y = float(self.vive.pose.pose.position.y)
        odomMsg.pose.pose.position.z = float(self.vive.pose.pose.position.z)

        odomMsg.pose.pose.orientation.w = float(self.imu.orientation.w)
        odomMsg.pose.pose.orientation.x = float(self.imu.orientation.x)
        odomMsg.pose.pose.orientation.y = float(self.imu.orientation.y)
        odomMsg.pose.pose.orientation.z = float(self.imu.orientation.z)

        odomMsg.twist.twist.linear.x = float(self.vive.twist.twist.linear.x)
        odomMsg.twist.twist.linear.y = float(self.vive.twist.twist.linear.y)
        odomMsg.twist.twist.linear.z = float(self.vive.twist.twist.linear.z)

        odomMsg.twist.twist.angular.x = float(self.imu.angular_velocity.x)
        odomMsg.twist.twist.angular.y = float(self.imu.angular_velocity.y)
        odomMsg.twist.twist.angular.z = float(self.imu.angular_velocity.z)

        self.odom_output.publish(odomMsg)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = self.car_id + '_base_link'

        t.transform.translation.x = float(self.vive.pose.pose.position.x)
        t.transform.translation.y = float(self.vive.pose.pose.position.y)
        t.transform.translation.z = float(self.vive.pose.pose.position.z)

        t.transform.rotation.w = float(self.imu.orientation.w)
        t.transform.rotation.x = float(self.imu.orientation.x)
        t.transform.rotation.y = float(self.imu.orientation.y)
        t.transform.rotation.z = float(self.imu.orientation.z)

        # Send the transformation
        self.br.sendTransform(t)

    def _process_imu_msg(self, msg):
        self.imu = msg

    def _process_vive_msg(self, msg):
        self.vive = msg


def main(args=None):
    rclpy.init(args=args)
    odom_pub = PublishOdom()
    rclpy.spin(odom_pub)
    odom_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
