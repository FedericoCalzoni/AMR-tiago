from threading import Thread
import rclpy
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

class TiagoArucoGrasp(Node):

    def __init__(self):
        super().__init__('tiago_aruco_grasp')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pose_pub = self.create_publisher(PoseStamped, "/target_pose_visualization", 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, "/ee_pose_visualization", 10)
        self.point_pub = self.create_publisher(Point, "target_point", 1)


        self.cam_K = None
        self.cam_D = None
        self.t_gripper_frame_to_camera = None
        self.move_state = "INIT"

        self.camera_frame = "head_front_camera_rgb_optical_frame"
        self.robot_base_frame = "base_link"
        self.approach_frame = "aruco_marker_frame_approach"
        self.gripper_frame = "aruco_marker_frame_grasp"
        self.ee_frame = "arm_tool_link"

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
        # self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.planner_id = "PRMstarkConfigDefault"
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
        
        self.create_timer(1.0, self.state_machine)
    
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
            point.positions = [0.09, 0.09]  # Open position
        else:
            point.positions = [0.035, 0.035]  # Closed position
        
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        # Add the point to the trajectory message
        msg.points.append(point)
        
        # Use action client for more reliable execution
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = msg
        
        # Send the goal and wait for it to be accepted
        goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        
        # Add a timeout to prevent hanging
        if not rclpy.spin_until_future_complete(self, goal_future, timeout_sec=2.0):
            self.get_logger().error("❌ Gripper goal timed out")
            return False
        
        goal_handle = goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("❌ Gripper goal was rejected")
            return False
        
        # Get the result with a timeout
        result_future = goal_handle.get_result_async()
        if not rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0):
            self.get_logger().error("❌ Gripper action timed out waiting for result")
            return False
        
        result = result_future.result()
        return result.result.error_code == 0
                    
    def state_machine(self):
        """State machine to handle the grabbing sequence."""
        while rclpy.ok():
            
            self.get_logger().info(f"Current state: {self.move_state}")
            
            if self.move_state == "INIT":
                # Open gripper before approaching
                self.control_gripper(open=True)
                sleep(3.0)
                # move arm to a confortable positon
                if self.move_to_pose(pos=self.default_pose['Position'], quat=self.default_pose['Orientation']):
                    self.move_state = "APPROACH"
                sleep(3.0)
                
            elif self.move_state == "APPROACH":
                if self.move_to_frame(self.approach_frame):
                    self.get_logger().info("Approach position reached, moving to GRASP state")
                    self.move_state = "GRASP"
                sleep(3.0)
                
            elif self.move_state == "GRASP":
                if self.move_to_frame(self.gripper_frame):
                    self.get_logger().info("Grasp position reached, closing gripper")
                    self.control_gripper(open=False)
                    self.move_state = "LIFT"
                sleep(3.0)
                
            elif self.move_state == "LIFT":
                if self.move_to_pose(pos=self.default_pose['Position'], quat=self.default_pose['Orientation']):
                    self.get_logger().info("Marker lifted, task complete!")
                    self.move_state = "DONE"
                sleep(3.0)
                    
            elif self.move_state == "DONE":
                self.get_logger().info("STATE MACHINE ENDED")
                break

def main(args=None):
    rclpy.init(args=args)

    tiago_move_node = TiagoArucoGrasp()

    rclpy.spin(tiago_move_node)
    tiago_move_node.destroy_node()
    
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
