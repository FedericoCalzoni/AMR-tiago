#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose
import math

class EnhancedNavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        
        # Action client with ReentrantCallbackGroup for parallel execution
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=ReentrantCallbackGroup()
        )
        
        # TF2 setup for dynamic goal positioning (merge-lab.pdf pg14-21)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Parameters from YAML config (merge-lab.pdf pg69-70)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('goal_tolerance', 0.15),
                ('planner_id', 'RRTstar'),
                ('controller_id', 'FollowPath'),
                ('max_nav_time', 300)
            ]
        )
        
        self.current_goal_handle = None
        self.get_logger().info("Navigation client initialized")

    async def navigate_to_frame(self, target_frame, offset=0.5):
        """
        Navigate to a TF frame with optional offset (merge-lab.pdf pg50-53)
        :param target_frame: TF frame ID to navigate to (e.g. 'aruco_marker_63')
        :param offset: Distance to maintain from target (meters)
        :return: True if successful, False otherwise
        """
        try:
            # Get transform from map to target frame
            transform = await self.tf_buffer.lookup_transform_async(
                'map',
                target_frame,
                rclpy.time.Time()
            )
            
            # Apply offset in the direction of the marker's orientation
            angle = 2 * math.atan2(
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Calculate offset position (merge-lab.pdf pg21)
            goal_pose.pose.position.x = transform.transform.translation.x - offset * math.cos(angle)
            goal_pose.pose.position.y = transform.transform.translation.y - offset * math.sin(angle)
            goal_pose.pose.orientation = transform.transform.rotation
            
            return await self.send_goal(goal_pose)
            
        except TransformException as ex:
            self.get_logger().error(f"TF lookup failed: {ex}")
            return False

    async def send_goal(self, pose):
        """Send navigation goal with configured parameters"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Set navigation parameters from config
        goal_msg.behavior_tree = self.get_parameter('planner_id').value
        goal_msg.controller_id = self.get_parameter('controller_id').value
        
        # Configure tolerance (merge-lab.pdf pg70)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal_checker.xy_goal_tolerance = self.get_parameter('goal_tolerance').value
        goal_msg.goal_checker.yaw_goal_tolerance = 0.15
        
        self.get_logger().info(f"Navigating to: {pose.pose.position}")
        
        # Send goal with async pattern
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        
        try:
            # Wait for goal acceptance with timeout
            await self._wait_with_timeout(send_goal_future, self.get_parameter('max_nav_time').value)
            self.current_goal_handle = send_goal_future.result()
            
            if not self.current_goal_handle.accepted:
                self.get_logger().error("Goal rejected!")
                return False
                
            # Wait for result
            result_future = self.current_goal_handle.get_result_async()
            await self._wait_with_timeout(result_future, self.get_parameter('max_nav_time').value)
            result = result_future.result().result
            
            if result.result_code == NavigateToPose.Result.SUCCESS:
                self.get_logger().info("Navigation succeeded!")
                return True
            else:
                self.get_logger().error(f"Navigation failed: {result.result_code}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Navigation interrupted: {str(e)}")
            return False

    def _feedback_callback(self, feedback_msg):
        """Handle navigation feedback (merge-lab.pdf pg32)"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {feedback.distance_remaining:.2f}m",
            throttle_duration_sec=2
        )

    async def _wait_with_timeout(self, future, timeout):
        """Helper for async operations with timeout"""
        try:
            await rclpy.task.wait_for_future_complete(future, timeout)
            if not future.done():
                raise TimeoutError()
        except TimeoutError:
            self.get_logger().error("Action timed out!")
            future.cancel()
            raise

    def cancel_navigation(self):
        """Cancel current navigation task"""
        if self.current_goal_handle:
            self.get_logger().info("Canceling current goal...")
            future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
            return future.result()
        return None

    def destroy(self):
        """Cleanup resources"""
        if self._action_client:
            self._action_client.destroy()
        super().destroy_node()