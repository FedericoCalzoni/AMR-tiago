import time
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist

class InitialPositionNode(Node):
    def __init__(self):
        super().__init__('initial_position_node')

        self.odom_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.amcl_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.done_publisher = self.create_publisher(Bool, '/localization/done', 10)
        self.odometry_subscription = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.covariance_threshold = 0.10
        self.covariance_msg = PoseWithCovarianceStamped()
        self.covariance_values = np.zeros(36, dtype=float).tolist()
        self.tiago_pose = [0.0, 0.0, 0.0]
        self.tiago_orientation = [0.0, 0.0, 0.0, 0.0]
        self.get_logger().info('Waiting for the initial position...')
        rclpy.spin_once(self, timeout_sec=1)
        self.twist = Twist()

    def amcl_callback(self, msg):
        """Callback for AMCL pose updates."""
        self.amcl_position = msg.pose.pose.position
        self.amcl_orientation = msg.pose.pose.orientation
        self.covariance_msg.pose.covariance = msg.pose.covariance

    def odom_callback(self, odom_msg):
        """Callback for Odometry updates."""
        x_o = odom_msg.pose.pose.orientation.x
        y_o = odom_msg.pose.pose.orientation.y
        z_o = odom_msg.pose.pose.orientation.z
        w_o = odom_msg.pose.pose.orientation.w        
        cov = np.zeros(36, dtype=float).tolist()
        cov[0] = 0.30   # x variance
        cov[7] = 0.30   # y variance
        cov[35] = 1.0   # yaw variance
        
        self.covariance_values = cov
        self.tiago_orientation = [x_o, y_o, z_o, w_o]

    def publish_initial_pose(self):
        """Publish the initial pose of the robot."""
        rclpy.spin_once(self, timeout_sec=1)
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.pose.pose.position.x = float(self.tiago_pose[0])
        initial_pose_msg.pose.pose.position.y = float(self.tiago_pose[1])
        initial_pose_msg.pose.pose.position.z = float(self.tiago_pose[2])
        initial_pose_msg.pose.pose.orientation.x = float(self.tiago_orientation[0])
        initial_pose_msg.pose.pose.orientation.y = float(self.tiago_orientation[1])
        initial_pose_msg.pose.pose.orientation.z = float(self.tiago_orientation[2])
        initial_pose_msg.pose.pose.orientation.w = float(self.tiago_orientation[3])
        initial_pose_msg.pose.covariance = tuple(float(x) for x in self.covariance_values)
        self.amcl_pose_publisher.publish(initial_pose_msg)
        self.get_logger().info('Published initial pose')

    def localization(self):
        """Perform localization by spinning the robot."""
        while True:
            self.get_logger().info('Localization in progress...')
            self.spin()
            rclpy.spin_once(self, timeout_sec=1)
            if self.check_covariance():
                self.get_logger().info('Localization completed.')
                self.done_publisher.publish(Bool(data=True))
                break

    def check_covariance(self):
        """Check if the covariance is below the threshold."""
        covariance_values = self.covariance_msg.pose.covariance
        max_covariance = max(abs(x) for x in covariance_values)
        if max_covariance < self.covariance_threshold:
            self.get_logger().info('\033[92mCovariance below the threshold. Robot is localized.\033[0m')
            return True
        else:
            self.get_logger().warn(f"Covariance {max_covariance:.3f} above threshold {self.covariance_threshold:.3f}")
            return False
        
    def spin(self):
        """Spin the robot"""
        self.twist.angular.z = -1.0
        self.publisher.publish(self.twist)
        time.sleep(2)

    def stop(self):
        """Stop the robot."""
        vel_msg = Twist()
        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0.0
        self.publisher.publish(vel_msg)

def main():
    rclpy.init()
    initial_position_node = InitialPositionNode()
    time.sleep(3)
    initial_position_node.publish_initial_pose()
    initial_position_node.localization()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
