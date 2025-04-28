#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Bool
import sys
from time import sleep

class FoldArm(Node):
    def __init__(self, positions=None):
        self.positions = positions if positions else [0.0, 0.0, -3.0, 1.5, 0.0, 0.0, 0.0]  # Default to navigation position
        self.action_complete = False
        self.action_succeeded = False
        
        super().__init__('fold_arm')
        
        # Publisher for the done signal
        self.done_publisher = self.create_publisher(Bool, '/fold_arm/done', 10)
        
        # Action client for arm controller
        self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available after waiting')
            return
            
        self.get_logger().info('Action server available!')
        self.send_goal()
        
    def send_goal(self):
        # Create the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'arm_1_joint', 
            'arm_2_joint', 
            'arm_3_joint', 
            'arm_4_joint', 
            'arm_5_joint', 
            'arm_6_joint', 
            'arm_7_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 2  # Duration of 2 seconds
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'Sending goal: positions={self.positions}')
        
        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Add callback for goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle:
            self.get_logger().error('Goal rejected')
            self.action_complete = True
            return
            
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the action server')
            self.action_complete = True
            return
            
        self.get_logger().info('Goal accepted by the action server')
        
        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result()
        self.action_complete = True
        
        if result.status == 4:  # FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info('Arm movement succeeded!')
            self.action_succeeded = True
            # Publish done message
            self.done_publisher.publish(Bool(data=True))
        else:
            status_codes = {
                1: "REJECTED",
                2: "ABORTED", 
                3: "CANCELED"
            }
            status_str = status_codes.get(result.status, f"UNKNOWN ({result.status})")
            self.get_logger().error(f'Arm movement failed with status: {status_str}')
            
        # Additional result info
        if hasattr(result, 'error_code') and result.error_code:
            self.get_logger().error(f'Error code: {result.error_code}')
        if hasattr(result, 'error_string') and result.error_string:
            self.get_logger().error(f'Error string: {result.error_string}')
            
    def wait_for_completion(self, timeout_sec=10.0):
        """Wait for the action to complete with timeout"""
        start_time = self.get_clock().now()
        
        while not self.action_complete:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().warn(f'Timeout after {timeout_sec} seconds')
                return False
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return self.action_succeeded

def main(args=None):
    rclpy.init(args=args)
    
    # Define arm positions
    folded_position_navigation = [0.0, 0.0, -3.0, 1.5, 0.0, 0.0, 0.0]
    # folded_position_manipulation = [1.02, 0.50, -1.06, 0.52, 0.98, -0.22, -0.34]
    
    # Default to navigation position
    positions = folded_position_navigation
    
    # Check if custom positions are provided via command line
    if len(sys.argv) > 7:
        try:
            positions = [float(sys.argv[i]) for i in range(1, 8)]
        except ValueError:
            print("Invalid arguments. Usage: script.py [joint1_pos joint2_pos ... joint7_pos]")
            print("Using default folded position for navigation")
    
    max_attempts = 5
    attempt = 0
    success = False
    
    while not success and attempt < max_attempts:
        attempt += 1
        
        if attempt > 1:
            print(f"Retry attempt {attempt}/{max_attempts}...")
            # Create a new node for each attempt
            rclpy.shutdown()
            rclpy.init(args=args)
            
        node = FoldArm(positions)
        
        # Wait for up to 10 seconds for completion
        success = node.wait_for_completion(timeout_sec=10.0)
        
        # If we timed out, destroy the node and try again
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