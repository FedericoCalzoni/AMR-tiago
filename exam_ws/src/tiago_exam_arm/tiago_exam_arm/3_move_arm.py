from threading import Thread
import argparse
import rclpy
import subprocess
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.executors
from rclpy.node import Node
from pymoveit2 import MoveIt2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from rclpy.wait_for_message import wait_for_message
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from time import sleep
from std_msgs.msg import Bool


class TiagoArucoGrasp(Node):

    def __init__(self, action=None):
        super().__init__('tiago_aruco_grasp')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pose_pub = self.create_publisher(PoseStamped, "/target_pose_visualization", 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, "/ee_pose_visualization", 10)
        self.point_pub = self.create_publisher(Point, "target_point", 1)
        
        self.done_publisher = self.create_publisher(Bool, '/move_arm/done', 10)
        self.done_get_frames = self.create_publisher(Bool, '/move_arm/get_frames', 10)

        self.cam_K = None
        self.cam_D = None
        self.t_gripper_frame_to_camera = None
        self.move_state = "INIT"

        self.camera_frame = "head_front_camera_rgb_optical_frame"
        self.robot_base_frame = "base_link"
        self.approach_frame = "aruco_marker_frame_approach"
        self.gripper_frame = "aruco_marker_frame_grasp"
        self.ee_frame = "arm_tool_link"
        
        self.action = action
        
        if self.action == None:
            raise ValueError("Action not specified. Use --action argument to specify the action: PICK63, PICK582, PLACE63, PLACE582")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        JOINT_NAMES = [
            "torso_lift_joint",
            "arm_1_joint",
            "arm_2_joint",
            "arm_3_joint",
            "arm_4_joint",
            "arm_5_joint",
            "arm_6_joint",
            "arm_7_joint",
            "arm_tool_joint",
        ]
        
        # Define joint names for the gripper
        self.gripper_joint_names = ['gripper_right_finger_joint', 'gripper_left_finger_joint']
        
        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=self.robot_base_frame,
            end_effector_name=self.ee_frame,
            group_name="arm_torso",
            callback_group=callback_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        # self.moveit2.planner_id = "PRMstarkConfigDefault"
        # self.moveit2.planner_id = "TRRTkConfigDefault"
        
        # Action client for controlling gripper
        self.gripper_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/gripper_controller/follow_joint_trajectory'
        )
        
        self.gripper_action_client.wait_for_server() 
        
        self.default_pose = {
            'Position': [0.5, 0.0, 0.8],
            'Orientation': [0.0, 0.0, 0.0, 1.0]
        }
        
        executor = rclpy.executors.MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        # Start the appropriate process based on action type
        if self.action.startswith("PICK"):
            self.create_timer(1.0, self.check_and_start_pick)
        elif self.action.startswith("PLACE"):
            # For PLACE actions, start immediately without checking transforms
            self.timer = self.create_timer(1.0, self.state_machine_PLACE)
        else:
            raise ValueError("Invalid action specified. Use PICK63, PICK582, PLACE63, or PLACE582.")
                
    def check_and_start_pick(self):
        """Check if required frames are available and start the state machine when they are"""
        self.get_logger().info(f"Checking for required frames...")
        
        try:
            for frame in [self.approach_frame, self.gripper_frame]:
                self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1)
                )
            
            self.get_logger().info("All required frames are available! Starting the state machine.")
                
            # Start the pick state machine
            self.timer = self.create_timer(1.0, self.state_machine_PICK)
            
        except Exception as e:
            self.get_logger().warn(f"Not all frames available yet: {e}")
            self.get_logger().info("Will check again in 1 second...")
       
        
    def run_node_subprocess(self, package, node, args=None):
        cmd = ['ros2', 'run', package, node]

        if args:
            cmd.extend(args)

        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        return process
    
    def run_node(self, package, node, args=None):
        cmd = ['ros2', 'run', package, node]

        if args:
            cmd.extend([str(arg) for arg in args])
            
        process = subprocess.run(cmd)

        return process
    
    
    def move_to_pose(self, pos, quat):
        try:
            self.moveit2.move_to_pose(position=pos, quat_xyzw=quat)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Failed to move to pose: {e}")
            return False

    def move_to_frame(self, target_frame):
        try:
            t_target = self.tf_buffer.lookup_transform(
                self.robot_base_frame, 
                target_frame, 
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=2)
            )
            
            if t_target is None:
                self.get_logger().error("❌ Transform lookup returned None")
                return False

            pos = [
                t_target.transform.translation.x, 
                t_target.transform.translation.y, 
                t_target.transform.translation.z
            ]

            quat = [
                t_target.transform.rotation.x,
                t_target.transform.rotation.y,
                t_target.transform.rotation.z,
                t_target.transform.rotation.w
            ]

            self.get_logger().info(f"pos={pos}")
            self.get_logger().info(f"quat={quat}")

            # Publish target pose
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.robot_base_frame
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = pos[0]
            pose_msg.pose.position.y = pos[1]
            pose_msg.pose.position.z = pos[2]
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            self.pose_pub.publish(pose_msg)

            # Move to the target pose
            self.max_velocity = 0.3
            self.max_acceleration = 0.3
            self.moveit2.move_to_pose(position=pos, quat_xyzw=quat, cartesian=True)
            self.moveit2.wait_until_executed()

            # Check final end-effector pose
            ee_tf = self.tf_buffer.lookup_transform(
                self.robot_base_frame, 
                self.ee_frame, 
                rclpy.time.Time(), 
                rclpy.duration.Duration(seconds=1)
            )

            actual_pos = [
                ee_tf.transform.translation.x,
                ee_tf.transform.translation.y,
                ee_tf.transform.translation.z
            ]

            actual_quat = [
                ee_tf.transform.rotation.x,
                ee_tf.transform.rotation.y,
                ee_tf.transform.rotation.z,
                ee_tf.transform.rotation.w
            ]

            ee_pose = PoseStamped()
            ee_pose.header.frame_id = self.robot_base_frame
            ee_pose.header.stamp = self.get_clock().now().to_msg()
            ee_pose.pose.position.x = actual_pos[0]
            ee_pose.pose.position.y = actual_pos[1]
            ee_pose.pose.position.z = actual_pos[2]
            ee_pose.pose.orientation.x = actual_quat[0]
            ee_pose.pose.orientation.y = actual_quat[1]
            ee_pose.pose.orientation.z = actual_quat[2]
            ee_pose.pose.orientation.w = actual_quat[3]

            self.ee_pose_pub.publish(ee_pose)

            self.get_logger().info('✅ Motion completed!')
            return True

        except Exception as e:
            self.get_logger().error(f"❌ Failed in move_to_frame: {e}")
            return False
                    
    def state_machine_PICK(self):
        """State machine to handle the grabbing sequence."""
        while rclpy.ok():
            
            self.get_logger().info(f"Current state: {self.move_state}")
            
            if self.move_state == "INIT":
                # Open gripper before approaching
                self.run_node('link_attacher_client', 'gripper_control', args=['--input_string', 'OPEN'])
                sleep(3.0)
                # move arm to a confortable positon
                # self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', args=['1.02',' 0.50', '-1.06',' 0.52', '0.98', '-0.22', '-0.34'])
                self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', args=['2.0',' 0.50', '1.0',' 0.52', '0.98', '-0.22', '-0.34'])
                self.done_get_frames.publish(Bool(data=True))
                self.move_state = "APPROACH"
                
            elif self.move_state == "APPROACH":
                if self.move_to_frame(self.approach_frame):
                    self.get_logger().info("Approach position reached, moving to GRASP state")
                    self.move_state = "GRASP"
                sleep(3.0)
                
            elif self.move_state == "GRASP":
                if self.move_to_frame(self.gripper_frame):
                    self.get_logger().info("Grasp position reached, closing gripper")
                    
                    if self.action == "PICK63":
                        self.run_node('link_attacher_client', 'gripper_control', args=['--input_string', 'CLOSE63'])
                    elif self.action == "PICK582":
                        self.run_node('link_attacher_client', 'gripper_control', args=['--input_string', 'CLOSE582'])
                    else:
                        raise ValueError("Invalid action specified. Use PICK63 or PICK582.")
                    self.move_state = "TRANSPORT"
                self.get_logger().info("Gripper closed, lifting marker")
                
            # elif self.move_state == "LIFT":
            #     if self.move_to_frame(self.approach_frame):
            #         self.get_logger().info("LIFT position reached, moving to TRANSPORT state")
            #         self.move_state = "TRANSPORT"
            #     sleep(3.0)
                
            elif self.move_state == "TRANSPORT":
                self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', args=['2.0',' 0.50', '1.0',' 0.52', '0.98', '-0.22', '-0.34'])
                self.get_logger().info("Marker lifted, task complete!")
                self.move_state = "DONE"
                sleep(3.0)
                    
            elif self.move_state == "DONE":
                self.get_logger().info("STATE MACHINE ENDED")
                self.done_publisher.publish(Bool(data=True))
                self.timer.cancel() 
                break
            
    def state_machine_PLACE(self):
        """State machine to handle the grabbing sequence."""
        while rclpy.ok():
            
            self.get_logger().info(f"Current state: {self.move_state}")
            
            if self.move_state == "INIT":
            #     self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm')
            #     sleep(10.0)
            #     self.move_state = "HOVER_THE_TABLE"
                
            # elif self.move_state == "HOVER_THE_TABLE":
                # self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', args=['1.02','0.50', '-1.06',' 0.52', '0.98', '-0.22', '-0.34'])
                # self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', args=['2.0',' 0.50', '1.0',' 0.52', '0.98', '-0.22', '-0.34'])
                # self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', args=['0.0',' 0.50', '1.0',' 1.0', '0.98', '-0.22', '-0.34'])
                
                if self.action == "PLACE63":
                    self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', args=['2.30', '0.40', '0.65', '1.01', '-0.91', '1.10', '0.06']
)
                elif self.action == "PLACE582":
                    self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', args=['2.08', '0.53', '0.71', '1.33', '-0.84', '1.01', '-0.44']
)
                else:
                    raise ValueError("Invalid action specified. Use PLACE63 or PLACE582.")

                sleep(1.0)
                self.move_state = "RELEASE"
                sleep(3.0)
                
            elif self.move_state == "RELEASE":
                self.run_node('link_attacher_client', 'gripper_control', args=['--input_string', 'OPEN'])
                self.move_state = "LIFT"
                self.get_logger().info("Gripper open, lifting arm")
                
            elif self.move_state == "LIFT":
                self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', args=['2.0',' 0.50', '1.0',' 0.52', '0.98', '-0.22', '-0.34'])
                self.get_logger().info("LIFT position reached")
                self.move_state = "DONE"
                    
            elif self.move_state == "DONE":
                self.get_logger().info("STATE MACHINE ENDED")
                self.done_publisher.publish(Bool(data=True))
                self.timer.cancel() 
                break

def main(args=None):
    rclpy.init(args=args)

    # USAGE: ros2 run tiago_exam_arm 3_move_arm --action PICK63
    
    parser = argparse.ArgumentParser(description='Tiago Aruco Grasp Node')
    parser.add_argument('--action', type=str, choices=['PICK63', 'PICK582', 'PLACE63', 'PLACE582'],
                        help='Action to perform: PICK63, PICK582, PLACE63, or PLACE582')
    parsed_args = parser.parse_args()
    
    tiago_move_node = TiagoArucoGrasp(action=parsed_args.action)

    rclpy.spin(tiago_move_node)
    tiago_move_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# def is_position_within_tolerance(self, actual_pos, target_pos, tolerance):
#         """Check if the end effector is within the specified tolerance of the target position."""
#         distance = sum((a - t) ** 2 for a, t in zip(actual_pos, target_pos)) ** 0.5
#         self.get_logger().info(f"Distance to target: {distance:.4f} m (tolerance: {tolerance:.4f} m)")
#         return distance <= tolerance

# if self.move_to_frame(self.approach_frame):
#     ee_tf = self.tf_buffer.lookup_transform(
#         self.robot_base_frame, 
#         self.ee_frame, 
#         rclpy.time.Time(), 
#         rclpy.duration.Duration(seconds=1)
#     )
    
#     if ee_tf is None:
#         self.get_logger().error("Could not get end effector transform")
#         sleep(3.0)
#         continue
    
#     actual_pos = [
#         ee_tf.transform.translation.x,
#         ee_tf.transform.translation.y,
#         ee_tf.transform.translation.z
#     ]
    
#     # Get the target approach frame position
#     approach_tf = self.tf_buffer.lookup_transform(
#         self.robot_base_frame, 
#         self.approach_frame, 
#         rclpy.time.Time(),
#         rclpy.duration.Duration(seconds=1)
#     )
    
#     if approach_tf is None:
#         self.get_logger().error("Could not get approach frame transform")
#         sleep(3.0)
#         continue
        
#     target_pos = [
#         approach_tf.transform.translation.x,
#         approach_tf.transform.translation.y,
#         approach_tf.transform.translation.z
#     ]
    
#     # Check if we're close enough
#     position_tolerance = 0.02  # 2 cm tolerance - adjust as needed
#     if self.is_position_within_tolerance(actual_pos, target_pos, position_tolerance):
#         self.get_logger().info("Approach position reached within tolerance, moving to GRASP state")
#         self.move_state = "GRASP"
#     else:
#         self.get_logger().warning("Not close enough to approach frame, retrying approach")

# sleep(3.0)