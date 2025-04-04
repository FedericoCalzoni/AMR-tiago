from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import PyKDL as kdl
from geometry_msgs.msg import PoseStamped

class TiagoArucoGrasp(Node):

    def __init__(self):
        super().__init__('tiago_aruco_grasp')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pose_pub = self.create_publisher(PoseStamped, "/target_pose_visualization", 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, "/ee_pose_visualization", 10)

        #self.timer_target = self.create_timer(0.1, self.timer_target)

        self.robot_base_frame = "base_link"
        self.target_frame = "aruco_marker_frame_approach"
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
        # self.moveit2.planner_id = "PRMstarkConfigDefault"
        self.moveit2.planner_id = "TRRTkConfigDefault"
        
        # self.moveit2.set_goal_position_tolerance(0.01)
        # self.moveit2.set_goal_orientation_tolerance(0.01)

        # Spin the node in background thread(s) and wait a bit for initialization
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        self.create_rate(1.0).sleep()

        self.move_to_pose()

    def move_to_pose(self):

        t_target = self.tf_buffer.lookup_transform(self.robot_base_frame, 
                                                   self.target_frame, 
                                                   rclpy.time.Time(),
                                                   rclpy.duration.Duration(seconds=2))
        if t_target is not None:
            
            pos = [t_target.transform.translation.x, 
                   t_target.transform.translation.y, 
                   t_target.transform.translation.z]
            
            quat = [t_target.transform.rotation.x,
                    t_target.transform.rotation.y,
                    t_target.transform.rotation.z,
                    t_target.transform.rotation.w]
            
            self.get_logger().info(f"pos={pos}")
            self.get_logger().info(f"quat={quat}")
            
            
            # Show the pose in RVIZ            
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

            # Move to pose
            self.max_velocity = 0.3
            self.max_acceleration = 0.3
            self.moveit2.move_to_pose(position=pos, quat_xyzw=quat, cartesian=False)
            self.moveit2.wait_until_executed()
            
            # After movement
            # Get the end effector pose
            ee_tf = self.tf_buffer.lookup_transform(self.robot_base_frame, self.ee_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=1))

            actual_pos = [ee_tf.transform.translation.x,
                        ee_tf.transform.translation.y,
                        ee_tf.transform.translation.z]

            actual_quat = [ee_tf.transform.rotation.x,
                        ee_tf.transform.rotation.y,
                        ee_tf.transform.rotation.z,
                        ee_tf.transform.rotation.w]

            # Publish the actual EE pose
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
            
            self.get_logger().info(f'✅ Motion completed!')

def main(args=None):
    rclpy.init(args=args)

    tiago_move_node = TiagoArucoGrasp()

    rclpy.spin(tiago_move_node)
    tiago_move_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
