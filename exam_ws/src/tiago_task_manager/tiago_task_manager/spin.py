# go_to_center_and_spin.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SpinAtCenter(Node):
    def __init__(self):
        super().__init__('spin_at_center')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.rotate)
        self.counter = 0

    def rotate(self):
        twist = Twist()
        twist.angular.z = 0.5  # rotazione oraria
        self.publisher.publish(twist)
        self.counter += 1

        if self.counter > 50:
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().info("Rotation complete.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SpinAtCenter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
