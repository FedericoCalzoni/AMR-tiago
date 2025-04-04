#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from time import sleep
from moveit_msgs.msg import MoveItErrorCodes, RobotState, Constraints, PositionIKRequest
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration

class ArucoGrabController(Node):
    def __init__(self):
        super().__init__('aruco_grab_controller')
        
        # TF Listener to receive the transformation from the ArUco marker
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Our move sequence state
        self.move_state = "INIT"  # States: INIT, APPROACH, GRASP, LIFT, DONE
        self.last_marker_transform = None
        
        # Define joint names for the arm
        self.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]
        
        # Define joint names for the gripper
        self.gripper_joint_names = ['gripper_right_finger_joint', 'gripper_left_finger_joint']

        
        # Publisher for joint trajectory commands to the arm
        self.arm_pub = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )
        
        # Publisher for joint trajectory commands to the gripper
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        # Action client for controlling arm movements
        self.arm_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Action client for controlling gripper
        self.gripper_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/gripper_controller/follow_joint_trajectory'
        )
        
        # Wait for action servers to be available
        if not self.arm_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm action server not available')
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available')
        
        # Subscribe to the transformation updates at a fixed rate
        self.timer = self.create_timer(1.0, self.state_machine)
        
        # Create a client to MoveIt's compute_ik service for motion planning
        self.moveit_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )
        
        while not self.moveit_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt service...')
        
        # Subscribe to current joint states
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info("✅ ArucoGrabController node initialized!")

    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        self.joint_state = msg
    
    def state_machine(self):
        """State machine to handle the grabbing sequence."""
        self.get_logger().info(f"Current state: {self.move_state}")
        
        if self.move_state == "INIT":
            # Try to locate the ArUco marker
            if self.locate_aruco_marker():
                self.get_logger().info("ArUco marker found, moving to APPROACH state")
                # Open gripper before approaching
                self.control_gripper(open=True)
                sleep(1.0)  # Wait for gripper to open
                self.move_state = "APPROACH"
            
        elif self.move_state == "APPROACH":
            # Move to the approach position
            if self.move_to_position('aruco_marker_frame_approach'):
                self.get_logger().info("Approach position reached, moving to GRASP state")
                sleep(1.0)  # Give time to stabilize
                self.move_state = "GRASP"
            
        elif self.move_state == "GRASP":
            # Move closer to grasp the marker
            if self.move_to_position('aruco_marker_frame_grasp'):
                self.get_logger().info("Grasp position reached, closing gripper")
                sleep(1.0)  # Give time to stabilize
                # Close gripper
                self.control_gripper(open=False)
                sleep(1.0)  # Wait for gripper to close
                self.move_state = "LIFT"
            
        elif self.move_state == "LIFT":
            # Lift the marker up
            if self.lift_marker():
                self.get_logger().info("Marker lifted, task complete!")
                self.move_state = "DONE"
                
        elif self.move_state == "DONE":
            pass  # Nothing more to do
    
    def locate_aruco_marker(self):
        """Attempts to locate the ArUco marker from TF."""
        try:
            # Try to get the transform from target_locked
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                'aruco_marker_frame_target',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5)
            )
            
            self.last_marker_transform = transform
            return True
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not find ArUco marker: {e}")
            return False
    
    def move_to_position(self, target_frame):
        """
        Moves the arm to a target position specified by a TF frame.
        Uses MoveIt to plan and execute the trajectory.
        
        Args:
            target_frame: The name of the TF frame to move to
            
        Returns:
            bool: True if the movement was successful, False otherwise
        """
        try:
            # Get the transform from base_link to the target frame
            transform = self.tf_buffer.lookup_transform(
                'base_link', 
                target_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5)
            )
            
            # Create a PoseStamped message for the target
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'base_link'
            target_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Set the position from the transform
            target_pose.pose.position.x = transform.transform.translation.x
            target_pose.pose.position.y = transform.transform.translation.y
            target_pose.pose.position.z = transform.transform.translation.z
            
            # Set the orientation from the transform
            target_pose.pose.orientation.x = transform.transform.rotation.x
            target_pose.pose.orientation.y = transform.transform.rotation.y
            target_pose.pose.orientation.z = transform.transform.rotation.z
            target_pose.pose.orientation.w = transform.transform.rotation.w
            
            self.get_logger().info(f"🔄 Moving to position: {target_frame}")
            
            # Send the pose command to MoveIt2
            result = self.send_pose_goal(target_pose)
            
            if result:
                self.get_logger().info(f"✅ Successfully moved to {target_frame}")
                return True
            else:
                self.get_logger().error(f"❌ Failed to move to {target_frame}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"⚠️ Error moving to {target_frame}: {e}")
            return False
    
    def lift_marker(self):
        """Lifts the marker after grasping by moving 20cm up."""
        try:
            if self.last_marker_transform is None:
                raise Exception("No marker transform available")
            
            # Create a PoseStamped message for the lift position
            lift_pose = PoseStamped()
            lift_pose.header.frame_id = 'base_link'
            lift_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Copy position from the last marker transform but add 20cm in Z
            lift_pose.pose.position.x = self.last_marker_transform.transform.translation.x
            lift_pose.pose.position.y = self.last_marker_transform.transform.translation.y
            lift_pose.pose.position.z = self.last_marker_transform.transform.translation.z + 0.2
            
            # Keep the same orientation
            lift_pose.pose.orientation.x = self.last_marker_transform.transform.rotation.x
            lift_pose.pose.orientation.y = self.last_marker_transform.transform.rotation.y
            lift_pose.pose.orientation.z = self.last_marker_transform.transform.rotation.z
            lift_pose.pose.orientation.w = self.last_marker_transform.transform.rotation.w
            
            self.get_logger().info("🔄 Lifting marker up")
            
            # Send the pose command to MoveIt2
            result = self.send_pose_goal(lift_pose)
            
            if result:
                self.get_logger().info("✅ Successfully lifted marker")
                return True
            else:
                self.get_logger().error("❌ Failed to lift marker")
                return False
                
        except Exception as e:
            self.get_logger().error(f"⚠️ Error lifting marker: {e}")
            return False
    
    def send_pose_goal(self, pose_stamped):
        """
        Sends a pose goal to move the arm using MoveIt IK.
        Uses a synchronous approach for easier debugging.
        
        Args:
            pose_stamped: A PoseStamped message with the target pose
            
        Returns:
            bool: True if the movement was successful, False otherwise
        """
        try:
            # Log the target position
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            z = pose_stamped.pose.position.z
            self.get_logger().info(f"📍 Moving to pose: x={x}, y={y}, z={z}")
            
            # Wait for joint state to be available - using a simple blocking approach
            attempt = 0
            while self.joint_state is None and attempt < 10:
                self.get_logger().info("Waiting for joint_state...")
                sleep(0.5)
                attempt += 1
                
            if self.joint_state is None:
                self.get_logger().error("❌ No joint state available")
                return False

            # Create IK request
            request = PositionIKRequest()
            request.group_name = "arm"
            request.pose_stamped = pose_stamped
            request.ik_link_name = "arm_tool_link"
            
            # Set the current joint state as the seed state
            robot_state = RobotState()
            robot_state.joint_state.name = self.joint_state.name
            robot_state.joint_state.position = self.joint_state.position
            request.robot_state = robot_state
            
            request.avoid_collisions = False #TODO: set to True
            request.constraints = Constraints()
            
            # Send request to MoveIt and wait for result (synchronous)
            self.get_logger().info("🔄 Sending IK request to MoveIt...")
            response = self.moveit_client.call(request)
            self.get_logger().info("🔄 Received IK response")
            
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(f"❌ IK solution failed with error code: {response.error_code.val}")
                return False
                
            # Extract joint positions from the solution
            joint_positions = []
            for joint_name in self.joint_names:
                idx = response.solution.joint_state.name.index(joint_name)
                joint_positions.append(response.solution.joint_state.position[idx])
                
            self.get_logger().info(f"✅ IK solution found: {joint_positions}")
            
            # Execute the motion using the action client in blocking mode
            goal_msg = FollowJointTrajectory.Goal()
            
            # Create trajectory
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = rclpy.duration.Duration(sec=2, nanosec=0)
            trajectory.points.append(point)
            
            # Set goal
            goal_msg.trajectory = trajectory
            
            # Send goal and wait for completion (synchronous approach)
            self.get_logger().info("🔄 Sending motion goal...")
            
            # Use send_goal_and_wait which is a synchronous call
            result = self.send_goal_and_wait(goal_msg)
            
            if not result or result.error_code != 0:
                self.get_logger().error(f"❌ Motion failed with error code: {result.error_code if result else 'unknown'}")
                return False
                
            self.get_logger().info("✅ Motion completed successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"⚠️ Error sending pose goal: {e}")
            return False

    def send_goal_and_wait(self, goal_msg):
        """
        Helper method to send a goal to the action server and wait for the result synchronously.
        
        Args:
            goal_msg: The goal message to send
            
        Returns:
            The result of the action, or None if it failed
        """
        # Using the synchronous action client interface
        goal_handle = self.arm_action_client.send_goal(goal_msg)
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            return None
        
        # This blocks until the goal is completed
        result = goal_handle.get_result()
        return result
        
    def control_gripper(self, open=True):
        """Controls the gripper (open or close)."""
        self.get_logger().info(f"👐 {'Opening' if open else 'Closing'} gripper")
        
        # Create a JointTrajectory message for the gripper
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names
        
        # Define a trajectory point with the desired gripper position
        point = JointTrajectoryPoint()
        
        # Set positions based on whether we want to open or close
        if open:
            point.positions = [0.04, 0.04]  # Open position
        else:
            point.positions = [0.01, 0.01]  # Closed position
        
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        # Add the point to the trajectory message
        msg.points.append(point)
        
        # Use action client for more reliable execution
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = msg
    
        # Send goal and continue without waiting for the full result
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, send_goal_future)
        # goal_handle = send_goal_future.result()

        # if not goal_handle.accepted:
        #     self.get_logger().error("❌ Gripper goal rejected")
        #     return False
            
        # self.get_logger().info("✅ Gripper goal accepted, continuing execution")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = ArucoGrabController()

    # Keep processing until done
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()