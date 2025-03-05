#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import numpy as np

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        
        # Create action client for NavigateToPose
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Navigation client initialized. Waiting for server...")
        
        # Wait for the action server to start
        self.action_client.wait_for_server()
        self.get_logger().info("Connected to navigation server!")

    def send_goal(self, x, y, theta=0.0, frame_id='map'):
        """Send a navigation goal to (x, y) with orientation theta (radians)."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = theta  # Simplified orientation (yaw)
        goal_pose.pose.orientation.w = 1.0    # No rotation (quaternion format)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Sending goal: x={x}, y={y}, theta={theta}")
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted!")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Handle the final result."""
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # Example coordinates (replace with your pick/place locations)
    pick_location = (0.7, -4, np.pi/5)  # (x, y, theta)
    #pick_location = (0.55, -2.6, 0.9)  # (x, y, theta)
    #place_location = (-1.35, -1.35, 0.2)
    
    nav_client = NavigationClient()
    
    # Send the first goal (pick location)
    nav_client.send_goal(*pick_location)
    
    # Spin to execute the goal
    rclpy.spin(nav_client)

if __name__ == '__main__':
    main()