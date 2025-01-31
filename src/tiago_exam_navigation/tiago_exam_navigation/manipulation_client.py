#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from pymoveit2 import MoveIt2
from pymoveit2 import GripperInterface
from linkattacher_msgs.srv import AttachLink, DetachLink
import math

class ManipulationClient(Node):
    def __init__(self):
        super().__init__('manipulation_client')
        
        # Arm configuration (merge-lab.pdf pg28-31)
        self.arm_joints = [
            "torso_lift_joint",
            "arm_1_joint", "arm_2_joint", "arm_3_joint",
            "arm_4_joint", "arm_5_joint", "arm_6_joint",
            "arm_7_joint"
        ]
        
        # MoveIt2 integration (merge-lab.pdf pg31-35)
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.arm_joints,
            base_link_name="base_link",
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=ReentrantCallbackGroup()
        )
        
        # Gripper control (merge-lab.pdf pg34-35)
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=["gripper_left_finger_joint", "gripper_right_finger_joint"],
            open_gripper_joint_positions=[0.04, 0.04],
            closed_gripper_joint_positions=[0.0, 0.0],
            gripper_command_action_name="/gripper_controller/joint_trajectory"
        )
        
        # Collision object management
        self.collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        
        # Link attacher services (merge-lab.pdf pg54-57)
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        # Parameters from config
        self.declare_parameters(
            namespace='',
            parameters=[
                ('approach_height', 0.2),
                ('retreat_height', 0.3),
                ('grasp_offset', 0.05),
                ('object_height', 0.06)
            ]
        )

    async def pick_object(self, marker_frame, object_id="aruco_cube"):
        """Full pick sequence for aruco marker (merge-lab.pdf pg50-53)"""
        try:
            # 1. Move to approach position
            approach_pose = await self._get_offset_pose(marker_frame, z_offset=self.get_parameter('approach_height').value)
            if not await self._move_to_pose(approach_pose):
                return False
            
            # 2. Open gripper
            if not await self.gripper.open():
                self.get_logger().error("Gripper open failed!")
                return False
            
            # 3. Add collision object
            await self._add_collision_object(marker_frame, object_id)
            
            # 4. Descend to grasp position
            grasp_pose = await self._get_offset_pose(marker_frame, z_offset=self.get_parameter('grasp_offset').value)
            if not await self._move_to_pose(grasp_pose):
                return False
            
            # 5. Close gripper
            if not await self.gripper.close():
                self.get_logger().error("Gripper close failed!")
                return False
            
            # 6. Attach object to gripper (merge-lab.pdf pg55)
            await self._attach_object(object_id)
            
            # 7. Retreat with object
            return await self._move_to_pose(approach_pose)
            
        except Exception as e:
            self.get_logger().error(f"Pick failed: {str(e)}")
            return False

    async def place_object(self, place_frame, object_id="aruco_cube"):
        """Place object sequence"""
        try:
            # 1. Move to approach above place position
            approach_pose = await self._get_offset_pose(place_frame, z_offset=self.get_parameter('approach_height').value)
            if not await self._move_to_pose(approach_pose):
                return False
            
            # 2. Descend to place position
            place_pose = await self._get_offset_pose(place_frame, z_offset=self.get_parameter('grasp_offset').value)
            if not await self._move_to_pose(place_pose):
                return False
            
            # 3. Open gripper
            if not await self.gripper.open():
                return False
            
            # 4. Detach object (merge-lab.pdf pg55)
            await self._detach_object(object_id)
            
            # 5. Retreat
            return await self._move_to_pose(approach_pose)
            
        except Exception as e:
            self.get_logger().error(f"Place failed: {str(e)}")
            return False

    async def _get_offset_pose(self, target_frame, z_offset=0.2):
        """Calculate pose with vertical offset from target frame (merge-lab.pdf pg21)"""
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, self)
        
        try:
            transform = await tf_buffer.lookup_transform_async(
                "base_link",
                target_frame,
                rclpy.time.Time()
            )
            
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z + z_offset
            
            # Align with surface normal (face down)
            pose.pose.orientation.w = 0.707  # Face down orientation
            pose.pose.orientation.z = 0.707
            
            return pose
            
        except Exception as e:
            self.get_logger().error(f"TF error: {str(e)}")
            raise

    async def _move_to_pose(self, pose):
        """Safe movement with collision checking"""
        self.moveit2.move_to_pose(pose)
        await self.moveit2.wait_until_executed()
        return self.moveit2.last_execution_success

    async def _add_collision_object(self, marker_frame, object_id):
        """Add collision object for the aruco marker cube (merge-lab.pdf pg77)"""
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = marker_frame
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.06, 0.06, 0.06]  # Cube dimensions
        
        co.primitives.append(primitive)
        co.primitive_poses.append(Pose())  # Centered on marker
        
        self.collision_pub.publish(co)
        self.get_logger().info(f"Added collision object: {object_id}")

    async def _attach_object(self, object_id):
        """Attach object to gripper (merge-lab.pdf pg55)"""
        req = AttachLink.Request()
        req.model1_name = "tiago"
        req.link1_name = "gripper_left_finger_link"
        req.model2_name = object_id
        req.link2_name = "link"
        
        future = self.attach_client.call_async(req)
        await future
        return future.result().success

    async def _detach_object(self, object_id):
        """Detach object from gripper"""
        req = DetachLink.Request()
        req.model1_name = "tiago" 
        req.link1_name = "gripper_left_finger_link"
        req.model2_name = object_id
        req.link2_name = "link"
        
        future = self.detach_client.call_async(req)
        await future
        return future.result().success

    async def home_arm(self):
        """Move arm to safe home position"""
        home_positions = [
            0.0,  # torso_lift_joint
            0.0, 1.32, 0.0, 1.4, 0.0, -0.2, 0.0  # arm joints
        ]
        self.moveit2.move_to_configuration(home_positions)
        await self.moveit2.wait_until_executed()
        return self.moveit2.last_execution_success

    def destroy(self):
        """Cleanup resources"""
        self.moveit2.destroy()
        self.gripper.destroy()
        super().destroy_node()