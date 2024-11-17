import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RotateOnce(Node):

    def __init__(self):
        super().__init__('rotate_once')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Set rotation speed (angular velocity in radians per second)
        self.rotation_speed = 0.5  # Positive for counterclockwise, negative for clockwise
        
        # Set the duration for which the rotation should be active
        self.rotation_duration = 13.0 # 10.0  # seconds
        time.sleep(1.0)
        # Start the rotation
        self.start_rotation()

    def start_rotation(self):
        twist = Twist()

        # Set the angular velocity for rotation
        twist.angular.z = self.rotation_speed

        # Publish the Twist message
        self.publisher_.publish(twist)
        self.get_logger().info(f'Rotating with angular velocity: {self.rotation_speed} rad/s for {self.rotation_duration} seconds')

        self.create_timer(self.rotation_duration, self.stop_rotation)

    def stop_rotation(self):
        twist = Twist()
o
        twist.angular.z = 0.0

        # Publish the stop command
        self.publisher_.publish(twist)
        self.get_logger().info('Rotation stopped.')



def main(args=None):
    rclpy.init(args=args)
    rotate_once = RotateOnce()
    rclpy.spin(rotate_once)

if __name__ == '__main__':
    main()
