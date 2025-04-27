import rclpy
import argparse
import sys
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

from linkattacher_msgs.srv import AttachLink, DetachLink

class GripperController(Node):
    def __init__(self, model=None, open_gripper=True):
        super().__init__('gripper_controller')
        
        # Action state tracking
        self.action_complete = False
        self.action_succeeded = False
        
        # Store command parameters
        self.open_gripper = open_gripper
        
        # Setup gripper action client
        self.gripper_joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        self.gripper_action_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        # Setup service clients for attach/detach
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')

        # Model & link names
        self.model1 = 'tiago'
        self.link1 = 'gripper_left_finger_link'
        self.link2 = 'link'
        
        self.model2 = None
        
        if model == '582':
            self.model2 = 'aruco_cube_exam_id582'
        elif model == '63':
            self.model2 = 'aruco_cube_exam_id63'
            
        # Initialize services and action client
        if not self.initialize_services():
            self.get_logger().error("Failed to initialize services - cannot continue")
            return
            
        # Execute the gripper control
        self.execute_gripper_control()
        
    def initialize_services(self):
        """Initialize all required services with timeout"""
        for client, name in [(self.attach_client, 'ATTACHLINK'), (self.detach_client, 'DETACHLINK')]:
            self.get_logger().info(f'⏳ Waiting for /{name} service...')
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"❌ Service /{name} not available after timeout")
                return False
            
            if not client.service_is_ready():
                self.get_logger().error(f"❌ Service /{name} is not ready")
                return False
            
            self.get_logger().info(f"✅ Service /{name} is ready")
            
        # Wait for action server
        self.get_logger().info('⏳ Waiting for gripper action server...')
        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("❌ Gripper action server not available after timeout")
            return False
            
        self.get_logger().info("✅ Gripper action server is ready")
        return True

    def execute_gripper_control(self):
        """Execute the gripper control action"""
        self.get_logger().info(f"👐 {'Opening' if self.open_gripper else 'Closing'} gripper")
        
        # Create a JointTrajectory message for the gripper
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.gripper_joint_names

        point = JointTrajectoryPoint()
        
        #  The maxium gripper opening is 0.065 m but it gives issues,
        #  so we set it to 0.060 m
        #  Close gripper to 0.030 m since aruco_cube_exam_id582 is 0.06 m
        point.positions = [0.064, 0.064] if self.open_gripper else [0.037, 0.037]
        point.time_from_start = Duration(sec=1, nanosec=0)
        goal_msg.trajectory.points = [point]

        self.get_logger().info(f"Sending goal: positions={point.positions}")
        
        # Send the goal
        self._send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        
        # Add callback for goal response
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback for handling goal response"""
        goal_handle = future.result()
        
        if not goal_handle:
            self.get_logger().error('❌ Goal rejected')
            self.action_complete = True
            return
            
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by the action server')
            self.action_complete = True
            return
            
        self.get_logger().info('✅ Goal accepted by the action server')
        
        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Callback for handling action result"""
        result = future.result()
        self.action_complete = True
        
        if not result:
            self.get_logger().error('❌ Failed to get result from action server')
            return
            
        if result.status == 4:  # FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info('✅ Gripper movement succeeded!')
            self.action_succeeded = True
            
            # Handle attachment or detachment based on the action
            if self.open_gripper:
                self.detach_all_models()
            else:
                self.attach()
        else:
            status_codes = {
                1: "REJECTED",
                2: "ABORTED", 
                3: "CANCELED"
            }
            status_str = status_codes.get(result.status, f"UNKNOWN ({result.status})")
            self.get_logger().error(f'❌ Gripper movement failed with status: {status_str}')
            
        # Only log error information if there's an actual error code
        if hasattr(result.result, 'error_code') and result.result.error_code != 0:
            self.get_logger().error(f'Error code: {result.result.error_code}')
            if hasattr(result.result, 'error_string') and result.result.error_string:
                self.get_logger().error(f'Error string: {result.result.error_string}')
        # Success message could be in the error_string field
        elif hasattr(result.result, 'error_string') and result.result.error_string and "successfully" in result.result.error_string.lower():
            self.get_logger().info(f'Success message: {result.result.error_string}')

    def wait_for_completion(self, timeout_sec=10.0):
        """Wait for the action to complete with timeout"""
        start_time = self.get_clock().now()
        
        while not self.action_complete:
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().warn(f'⚠️ Timeout after {timeout_sec} seconds')
                return False
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return self.action_succeeded

    def attach(self):
        """Attach an object to the gripper"""
        if not self.model2:
            self.get_logger().info("⚠️ No model specified for attachment")
            return True
            
        self.get_logger().info(f"🔗 Attaching object {self.model2}...")
        req = AttachLink.Request()
        req.model1_name = self.model1
        req.link1_name = self.link1
        req.model2_name = self.model2
        req.link2_name = self.link2
        
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error("❌ Attach service call timed out")
            return False
            
        result = future.result()
        if result and result.success:
            self.get_logger().info("✅ Object attached successfully")
            return True
        else:
            self.get_logger().error("❌ Failed to attach object")
            return False

    def detach(self, model_name=None):
        """Detach an object from the gripper"""
        target_model = model_name if model_name else self.model2
        
        if not target_model:
            self.get_logger().info("⚠️ No model specified for detachment")
            return True
            
        self.get_logger().info(f"🧩 Detaching object {target_model}...")
        req = DetachLink.Request()
        req.model1_name = self.model1
        req.link1_name = self.link1
        req.model2_name = target_model
        req.link2_name = self.link2
        
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error("❌ Detach service call timed out")
            return False
            
        result = future.result()
        if result and result.success:
            self.get_logger().info("✅ Object detached successfully")
            return True
        else:
            self.get_logger().warning("⚠️ Failed to detach object - may not have been attached")
            return True  # Don't fail on detach errors as it might not be attached
            
    def detach_all_models(self):
        """Detach all possible models to ensure clean state when opening gripper."""
        self.get_logger().info("🧩 Detaching all potential objects...")
        
        # Try to detach model 63
        self.detach("aruco_cube_exam_id63")
        
        # Try to detach model 582
        self.detach("aruco_cube_exam_id582")
        
        return True

def main(args=None):
    
    # USAGE: ros2 run link_attacher_client gripper_control --input_string OPEN
    
    
    parser = argparse.ArgumentParser(description='Gripper controller Node')
    parser.add_argument('--input_string', type=str, required=True, 
                        help='possible values: "OPEN", "CLOSE63", or "CLOSE582"')
    parsed_args, other_args = parser.parse_known_args()
    
    rclpy.init(args=other_args)
    
    command = parsed_args.input_string.strip().upper()
    
    # Validate command
    if command not in ["OPEN", "CLOSE63", "CLOSE582"]:
        print(f"Invalid input_string: {parsed_args.input_string} (use 'OPEN', 'CLOSE63', 'CLOSE582')")
        rclpy.shutdown()
        return 1
    
    # Parse command
    open_gripper = (command == "OPEN")
    model = None
    if command == "CLOSE63":
        model = "63"
    elif command == "CLOSE582":
        model = "582"
    
    # Add retry mechanism similar to move_head_to_pose.py
    max_attempts = 5
    attempt = 0
    success = False
    
    while not success and attempt < max_attempts:
        attempt += 1
        
        if attempt > 1:
            print(f"Retry attempt {attempt}/{max_attempts}...")
            # Create a new node for each attempt
            rclpy.shutdown()
            rclpy.init(args=other_args)
            
        node = GripperController(model=model, open_gripper=open_gripper)
        
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