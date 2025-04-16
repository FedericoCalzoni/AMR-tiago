from threading import Thread
import rclpy, subprocess
from rclpy.callback_groups import ReentrantCallbackGroup
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Bool
from pymoveit2 import MoveIt2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from time import sleep
from enum import Enum

class State(Enum):
    LOOKING_FOR_BOX = 0
    ALIGN_TO_BOX = 1
    DONE = 2

class StateMachineNavigation(Node):
    def __init__(self):
        super().__init__('tiago_state_controller')

        # Subscription to node termination topics
        self.create_subscription(Bool, '/nav_to_box/done', self.nav_to_box_callback, 10)
        self.create_subscription(Bool, '/align_to_box_face/done', self.align_to_box_face_callback, 10)
        self.node_termination = False 
        self.node_launched = False

        callback_group = ReentrantCallbackGroup()
        self.current_state = State.LOOKING_FOR_BOX
        self.robot_base_frame = "base_link"
        self.ee_frame = "arm_tool_link"
        
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
        self.moveit2.planner_id = "PRMstarkConfigDefault"
        
        # Action client for controlling gripper
        self.gripper_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/gripper_controller/follow_joint_trajectory'
        )
        
        self.gripper_action_client.wait_for_server() 
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Default positions
        self.closed_pose = {
            'Position': [0.3, 0.3, 0.3],
            'Orientation': [0.0, 0.0, 0.0, 0.0]
        }
        
        # Setup executor for background tasks
        executor = rclpy.executors.MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        # Run state machine
        self.get_logger().info('Starting state machine...')
        self.move_to_pose(self.closed_pose['Position'], self.closed_pose['Orientation'])
        self.state_machine_timer = self.create_timer(0.1, self.state_machine_step)
    
    def move_to_pose(self, pos, quat):
        """Move to a specific pose"""
        try:
            self.moveit2.move_to_pose(position=pos, quat_xyzw=quat)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Failed to move to pose: {e}")
            return False
    
    def nav_to_box_callback(self, msg):
        if msg.data:
            self.node_termination = True

    def align_to_box_face_callback(self, msg):
        if msg.data:
            self.node_termination = True

    def run_node(self, package, node):
        # Format: ros2 run <package_name> <node_executable>
        process = subprocess.Popen(
            ['ros2', 'run', package, node],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return process
    
    def stop_node(self, process):
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()

    def state_machine_step(self):
        #self.get_logger().info(f"Current state: {self.current_state.name}")
                
        if self.current_state == State.LOOKING_FOR_BOX:
            # Look for the box
            if not self.node_launched:
                self.get_logger().info("Looking for box...")
                self.navigation_process = self.run_node('tiago_exam_navigation', 'navigate_to_box')
                self.node_launched = True
                
            if self.node_termination:
                self.get_logger().info("Box found, transitioning to ALIGN_TO_BOX")
                self.current_state = State.ALIGN_TO_BOX
                self.node_termination = False
                self.node_launched = False
                
        elif self.current_state == State.ALIGN_TO_BOX:
            # Align to the detected box
            if not self.node_launched:
                self.get_logger().info("Aligning to box face...")
                self.align_process = self.run_node('tiago_exam_navigation', 'align_to_box_face')
                self.node_launched = True
                
            if self.node_termination:
                self.get_logger().info("Alignment complete, transitioning to DONE")
                self.current_state = State.DONE
                self.node_termination = False
                self.node_launched = False
                
        elif self.current_state == State.DONE:
            self.get_logger().info("STATE MACHINE COMPLETED")
            #self.stop_node(self.navigation_process)
            #self.stop_node(self.align_process)
            # You might want to cancel the timer here to stop further execution
            self.state_machine_timer.cancel()
            
        else:
            self.get_logger().error("Unknown state")

    def run_state_machine(self):
        """Main state machine loop"""
        while rclpy.ok():
            #self.get_logger().info(f"Current state: {self.current_state.name}")
            
            if self.current_state == State.CLOSING_ARM:
                # Close the gripper and move the arm to home position
                self.control_gripper(open=True)
                sleep(3.0)
                if self.move_to_pose(self.home_pose['Position'], self.home_pose['Orientation']):
                    self.current_state = State.LOOKING_FOR_BOX
                sleep(2.0)
                
            elif self.current_state == State.LOOKING_FOR_BOX:
                # Move to a position to look for the box
                if not self.node_launched:
                    navigation_to_box = self.run_node('tiago_exam_navigation', 'navigate_to_box')
                    self.get_logger().info("Looking for box...")
                    self.node_launched = True

                if self.node_termination:
                    self.current_state = State.ALIGN_TO_BOX
                    self.node_termination = False
                    self.node_launched = False
                    #self.stop_node(navigation_to_box)
                sleep(2.0)
                
            elif self.current_state == State.ALIGN_TO_BOX:
                # Align to the detected box
                if not self.node_launched:
                    align_to_box_face = self.run_node('tiago_exam_navigation', 'align_to_box_face')
                    self.get_logger().info("Aligning to box face...")
                    self.node_launched = True

                if self.node_termination:
                    self.current_state = State.DONE
                    self.node_termination = False
                    self.node_launched = False
                    #self.stop_node(align_to_box_face)
                sleep(2.0)
                
            elif self.current_state == State.DONE:
                self.get_logger().info("STATE MACHINE COMPLETED")
                break
            sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    tiago_state_controller = StateMachineNavigation()
    rclpy.spin(tiago_state_controller)
    tiago_state_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()