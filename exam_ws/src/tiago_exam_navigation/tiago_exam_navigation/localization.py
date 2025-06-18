import time
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

TWO_PI = 2 * np.pi
ROTATION_VELOCITY = -0.3

class InitialPositionNode(Node):
    def __init__(self):
        super().__init__('initial_position_node')

        self.odom_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, 10)

        self.amcl_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

        self.odometry_subscription = self.create_subscription(
            Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)

        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.covariance_threshold = 0.05
        self.covariance_msg = PoseWithCovarianceStamped()
        self.covariance_values = [0.0] * 36  # Initialize as a list

        self.tiago_pose = [0, 0, 0]
        self.tiago_orientation = [0, 0, 0, 0]
        
        self.get_logger().info('Waiting for the initial position...')
        rclpy.spin_once(self, timeout_sec=1)

    def amcl_callback(self, msg):
        self.amcl_position = msg.pose.pose.position
        self.amcl_orientation = msg.pose.pose.orientation
        self.covariance_msg.pose.covariance = msg.pose.covariance

    def odom_callback(self, odom_msg):
        x_o = odom_msg.pose.pose.orientation.x
        y_o = odom_msg.pose.pose.orientation.y
        z_o = odom_msg.pose.pose.orientation.z
        w_o = odom_msg.pose.pose.orientation.w
        
        # Initialize covariance with more reasonable values
        cov = [0.0] * 36
        cov[0] = 0.25   # x variance
        cov[7] = 0.25   # y variance
        cov[35] = 1.0   # yaw variance
        
        self.covariance_values = cov
        self.tiago_orientation = [x_o, y_o, z_o, w_o]

    def publish_initial_pose(self):
        rclpy.spin_once(self, timeout_sec=1)
        initial_pose_msg = PoseWithCovarianceStamped()
        
        # Set the frame ID first
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        initial_pose_msg.pose.pose.position.x = float(self.tiago_pose[0])
        initial_pose_msg.pose.pose.position.y = float(self.tiago_pose[1])
        initial_pose_msg.pose.pose.position.z = float(self.tiago_pose[2])
        
        # Set orientation
        initial_pose_msg.pose.pose.orientation.x = float(self.tiago_orientation[0])
        initial_pose_msg.pose.pose.orientation.y = float(self.tiago_orientation[1])
        initial_pose_msg.pose.pose.orientation.z = float(self.tiago_orientation[2])
        initial_pose_msg.pose.pose.orientation.w = float(self.tiago_orientation[3])
        
        # Set covariance
        initial_pose_msg.pose.covariance = tuple(float(x) for x in self.covariance_values)
        
        self.amcl_pose_publisher.publish(initial_pose_msg)
        self.get_logger().info('Published initial pose')

    def localization(self):
        while True:
            self.get_logger().info('Localization in progress...')
            self.rotate()
            rclpy.spin_once(self, timeout_sec=1)
            if self.check_covariance():
                self.get_logger().info('Localization completed.')
                break

    def check_covariance(self):
        covariance_values = self.covariance_msg.pose.covariance
        max_covariance = max(abs(x) for x in covariance_values)
        if max_covariance < self.covariance_threshold:
            self.get_logger().info("Covariance below the threshold.")
            self.get_logger().info("Robot is localized.")
            return True
        else:
            self.get_logger().warn(f"Covariance {max_covariance:.3f} above threshold {self.covariance_threshold:.3f}")
            return False

    def rotate(self):
        vel_msg = Twist()
        vel_msg.angular.z = ROTATION_VELOCITY
        self.publisher.publish(vel_msg)
        self.get_logger().info('Publishing: "%s"' % vel_msg)
        time.sleep(-TWO_PI / vel_msg.angular.z)
        self.stop()

    def stop(self):
        vel_msg = Twist()
        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0.0
        self.publisher.publish(vel_msg)
        self.get_logger().info('Publishing: "%s"' % vel_msg)

def main():
    rclpy.init()
    initial_position_node = InitialPositionNode()
    time.sleep(5)
    initial_position_node.publish_initial_pose()
    initial_position_node.localization()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
