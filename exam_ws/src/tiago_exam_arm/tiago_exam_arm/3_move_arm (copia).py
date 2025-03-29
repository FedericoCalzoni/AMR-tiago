from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TiagoArucoGrasp(Node):

    def __init__(self):
        super().__init__('tiago_aruco_grasp')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_base_frame = "base_link"
        self.target_frame = "aruco_marker_frame_approach"

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
            end_effector_name="gripper_grasping_frame",
            group_name="arm_torso",
            callback_group=callback_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Spin the node in background thread(s) and wait a bit for initialization
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        # Attendere l'inizializzazione del TF Listener
        self.get_logger().info("Attesa inizializzazione TF Listener...")
        self.create_rate(1.0).sleep()

        self.move_to_pose()

    def move_to_pose(self):

        timeout = 5  # Tempo massimo di attesa in secondi
        start_time = time.time()

        while not self.tf_buffer.can_transform(self.robot_base_frame, self.target_frame, rclpy.time.Time()):
            if time.time() - start_time > timeout:
                self.get_logger().error(f"Frame {self.target_frame} non trovato entro {timeout} secondi.")
                return
            self.get_logger().warn(f"Attesa del frame {self.target_frame}...")
            time.sleep(0.5)

        t_target = self.tf_buffer.lookup_transform(self.robot_base_frame, self.target_frame, rclpy.time.Time())

        pos = [t_target.transform.translation.x, t_target.transform.translation.y, t_target.transform.translation.z]
        quat = [t_target.transform.rotation.x,
                t_target.transform.rotation.y,
                t_target.transform.rotation.z,
                t_target.transform.rotation.w]
        
        # Move to pose
        self.moveit2.move_to_pose(position=pos, quat_xyzw=quat)
        self.moveit2.wait_until_executed()

def main(args=None):
    rclpy.init(args=args)

    tiago_move_node = TiagoArucoGrasp()

    rclpy.spin(tiago_move_node)
    tiago_move_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

