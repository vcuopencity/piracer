import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import math


class MotionController(Node):
    def __init__(self, max_speed, max_steering_angle):
        super().__init__('motion_controller')

        self.max_speed = max_speed  # maximum speed of the car
        self.max_steering_angle = max_steering_angle  # maximum steering angle of the car
        self.target_pos = None  # target position of the car, initially set to None

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # qos_profile.depth
        )

        self.target_subscription = self.create_subscription(
            PointStamped,
            '/target',
            self.target_callback,
            10
        )

        self.steering_publisher = self.create_publisher(Float32, '/steering', 10)
        self.speed_publisher = self.create_publisher(Float32, '/speed', 10)

    def target_callback(self, target):
        # Get the target position from the /target message
        self.target_pos = (target.point.x, target.point.y)

    def odom_callback(self, odom):
        # Get the car's current position and orientation from the /odom message
        car_pos = (odom.pose.pose.position.x, odom.pose.pose.position.y)
        car_orientation = odom.pose.pose.orientation.z

        # Get the car's current velocity from the /odom message
        car_vel = odom.twist.twist.linear.x

        # Calculate the steering angle and speed based on the current position, orientation, velocity and target position
        if self.target_pos is not None:
            steering_angle = self.calculate_steering_angle(car_pos, car_orientation, self.target_pos)
            desired_speed = self.calculate_speed(car_pos, self.target_pos)

            # Publish the steering angle and desired speed to their own topics
            self.steering_publisher.publish(Float32(data=steering_angle))
            self.speed_publisher.publish(Float32(data=desired_speed))

    def calculate_steering_angle(self, car_pos, car_orientation, target_pos):
        # Calculate the angle between the car's current position and the target position
        dx = target_pos[0] - car_pos[0]
        dy = target_pos[1] - car_pos[1]
        target_angle = math.atan2(dy, dx)

        # Calculate the steering angle needed to reach the target angle
        steering_angle = target_angle - car_orientation
        steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))

        return steering_angle

    def calculate_speed(self, car_pos, target_pos):
        # Calculate the distance to the target position
        dx = target_pos[0] - car_pos[0]
        dy = target_pos[1] - car_pos[1]
        distance_to_target = math.sqrt(dx**2 + dy**2)

        # Calculate the desired speed based on the distance to the target position
        desired_speed = self.max_speed * distance_to_target / 10.0  # adjust the division factor to set the desired speed

        # Limit the desired speed to the maximum speed of the car
        desired_speed = min(desired_speed, self.max_speed)

        return desired_speed


def main(args=None):
    rclpy.init(args=args)

    # Create a motion controller for a car with a maximum speed of 5 m/s and a maximum steering angle of 45 degrees
    motion_controller = MotionController(max_speed=5, max_steering_angle=math.radians(45))

    rclpy.spin(motion_controller)

    motion_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
