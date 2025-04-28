#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import argparse
import sys
from math import sin, cos, sqrt
from tf2_ros import TransformListener, Buffer


class NavigationClient(Node):
    def __init__(self, x, y, theta, frame_id='map'):
        self.x = x
        self.y = y
        self.theta = theta
        self.frame_id = frame_id
        self.action_complete = False
        self.action_succeeded = False
        self.position_tolerance = 0.5  # 10cm tolerance for position
        
        super().__init__('navigation_client')
        
        # Create action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Setup TF listener to get robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Wait for the action server to start
        self.get_logger().info("Waiting for navigation server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available after waiting')
            return
            
        self.get_logger().info("Connected to navigation server!")
        self.send_goal()

    def send_goal(self):
        """Send a navigation goal to (x, y) with orientation theta (radians)."""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = self.x
        goal_pose.pose.position.y = self.y
        goal_pose.pose.position.z = 0.0
        
        # Set orientation as quaternion
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = sin(self.theta / 2.0)
        goal_pose.pose.orientation.w = cos(self.theta / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Sending goal: x={self.x}, y={self.y}, theta={self.theta}")
        
        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Add callback for goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        
        if not goal_handle:
            self.get_logger().error('Goal rejected')
            self.action_complete = True
            return
            
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the navigation server')
            self.action_complete = True
            return
            
        self.get_logger().info('Goal accepted by the navigation server')
        
        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle the final result."""
        result = future.result()
        
        # Check if we received a valid result
        if result:
            # Verify if the robot is at the goal pose
            if self.verify_robot_at_goal_pose():
                self.get_logger().info('Navigation succeeded! Robot is at goal pose.')
                self.action_succeeded = True
            else:
                self.get_logger().error('Navigation reported success but robot is not at goal pose.')
        else:
            self.get_logger().error('Navigation failed')
            
        self.action_complete = True
    
    def verify_robot_at_goal_pose(self):
        """Verify if the robot's current pose matches the goal pose within tolerance."""
        try:
            # Get the latest transform from map to base_footprint
            transform = self.tf_buffer.lookup_transform(
                self.frame_id, 'base_footprint', rclpy.time.Time())
            
            # Extract position from transform
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            # Calculate distance between current position and goal
            distance = sqrt((current_x - self.x)**2 + (current_y - self.y)**2)
            
            self.get_logger().info(f"Current position: x={current_x}, y={current_y}")
            self.get_logger().info(f"Distance to goal: {distance} (tolerance: {self.position_tolerance})")
            
            # Check if within tolerance
            return distance <= self.position_tolerance
            
        except Exception as e:
            self.get_logger().error(f'Error getting robot pose: {e}')
            return False
    
    def wait_for_completion(self, timeout_sec=30.0):
        """Wait for the action to complete with timeout"""
        start_time = self.get_clock().now()
        
        while not self.action_complete:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().warn(f'Timeout after {timeout_sec} seconds')
                return False
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return self.action_succeeded


def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Navigation client')
    parser.add_argument('--goal', type=float, nargs=3, required=True,
                      help='Goal pose as [x, y, theta]')
    parsed_args = parser.parse_args(args=args)
    
    rclpy.init(args=args)
    
    # Extract goal pose
    x, y, theta = parsed_args.goal
    
    max_attempts = 50
    attempt = 0
    success = False
    
    while not success and attempt < max_attempts:
        attempt += 1
        
        if attempt > 1:
            print(f"Retry attempt {attempt}/{max_attempts}...")
            # Create a new node for each attempt
            rclpy.shutdown()
            rclpy.init(args=args)
            
        node = NavigationClient(x, y, theta)
        
        # Wait for up to 30 seconds for completion
        success = node.wait_for_completion(timeout_sec=30.0)
        
        # If we timed out or failed, destroy the node and try again
        if not success:
            node.get_logger().warn(f"Attempt {attempt} failed, " + 
                                  ("retrying..." if attempt < max_attempts else "giving up."))
            node.destroy_node()
        else:
            # Clean up on success
            node.destroy_node()
            
    # Return exit code based on success
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())