from threading import Thread
import rclpy, subprocess, os, sys
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Bool
import time, signal 
from enum import Enum

class State(Enum):
    FOLDING_ARM = 0
    LOOKING_FOR_BOX = 1
    ALIGN_TO_BOX = 2
    DONE = 3

class StateMachineNavigation(Node):
    def __init__(self):
        super().__init__('tiago_state_controller_navigation')
                
        # Publisher for completion message
        self.done_publisher = self.create_publisher(Bool, '/state_machine_navigation/done', 10)
        
        # Subscription to node termination topics
        self.create_subscription(Bool, '/nav_to_box/done', self.nav_to_box_callback, 10)
        self.create_subscription(Bool, '/align_to_box_face/done', self.align_to_box_face_callback, 10)
        self.create_subscription(Bool, '/fold_arm/done', self.fold_arm_callback, 10)
        
        # Subscription for reciving start command
        self.create_subscription(Bool, '/start_navigation_state_machine', self.start_callback, 10)
        
        self.node_termination = False
        self.box_detection_terminated = False 
        self.alignment_to_face_terminated = False
        self.node_launched = False
        self.started = False  # Flag to indicate if the state machine has started

        callback_group = ReentrantCallbackGroup()
        self.current_state = State.FOLDING_ARM
        
        # Setup executor for background tasks
        executor = rclpy.executors.MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        # Timer for the state machine when the start signal is received 
        self.state_machine_timer = None
        self.get_logger().info('Navigation state machine initialized and waiting for start command')

    def start_callback(self, msg):
        if msg.data and not self.started:
            self.get_logger().info('Received start command, beginning navigation state machine')
            self.started = True
            self.reset_state_machine()
            self.state_machine_timer = self.create_timer(0.1, self.state_machine_step)

    def reset_state_machine(self):
        # Reset all state variables
        self.current_state = State.FOLDING_ARM
        self.node_termination = False
        self.box_detection_terminated = False
        self.alignment_to_face_terminated = False
        self.node_launched = False
        
    def fold_arm_callback(self, msg):
        if msg.data:
            self.node_termination = True
    
    def nav_to_box_callback(self, msg):
        if msg.data:
            self.box_detection_terminated = True

    def align_to_box_face_callback(self, msg):
        if msg.data:
            self.alignment_to_face_terminated = True

    def run_node(self, package, node, input = ""):
        # Format: ros2 run <package_name> <node_executable>
        cmd = ['ros2', 'run', package, node]
        if input:
            input_args = input.split()
            cmd.extend(input_args)
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return process
    
    def state_machine_step(self):
        if self.current_state == State.FOLDING_ARM:
            # Fold the arm
            if not self.node_launched:
                self.get_logger().info("Folding arm...")
                self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm', '--in nav')
                self.node_launched = True
                
            if self.node_termination:
                self.get_logger().info("Arm folded, transitioning to LOOKING_FOR_BOX")
                self.current_state = State.LOOKING_FOR_BOX
                self.node_launched = False
                
        elif self.current_state == State.LOOKING_FOR_BOX:
            # Look for the box
            if not self.node_launched:
                self.get_logger().info("Looking for box...")
                self.navigation_process = self.run_node('tiago_exam_navigation', 'navigate_to_box')
                self.node_launched = True
                
            if self.box_detection_terminated:
                self.get_logger().info("Box found, transitioning to ALIGN_TO_BOX")
                self.current_state = State.ALIGN_TO_BOX
                self.node_launched = False
                
        elif self.current_state == State.ALIGN_TO_BOX:
            # Align to the detected box
            if not self.node_launched:
                self.get_logger().info("Aligning to box face...")
                self.align_process = self.run_node('tiago_exam_navigation', 'align_to_box_face')
                self.node_launched = True
                
            if self.alignment_to_face_terminated:
                self.get_logger().info("Alignment complete, transitioning to DONE")
                self.current_state = State.DONE
                self.node_termination = False
                self.node_launched = False
                
        elif self.current_state == State.DONE:
            self.get_logger().info("NAVIGATION STATE MACHINE COMPLETED")
            
           
            # Publish done message multiple times to ensure it is received
            for i in range(50):  # More iterations for better reliability
                done_msg = Bool()
                done_msg.data = True
                self.done_publisher.publish(done_msg)
                time.sleep(0.05)  # Short delay to ensure message is sent
            
            self.get_logger().info("Done messages published, stopping timer")
            self.state_machine_timer.cancel()
            self.started = False  # Reset the started flag
        else:
            self.get_logger().error("Unknown state")

def main(args=None):
    rclpy.init(args=args)
    navigation_state_machine = StateMachineNavigation()
    rclpy.spin(navigation_state_machine)
    navigation_state_machine.get_logger().info("Shutting down state machine navigation")    
    # Cleanup
    navigation_state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
