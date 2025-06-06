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
        super().__init__('tiago_state_controller')
                
        self.done_publisher = self.create_publisher(Bool, '/state_machine_navigation/done', 10)
        self.done_publisher.publish(Bool(data=False))
        
        # Subscription to node termination topics
        self.create_subscription(Bool, '/nav_to_box/done', self.nav_to_box_callback, 10)
        self.create_subscription(Bool, '/align_to_box_face/done', self.align_to_box_face_callback, 10)
        self.create_subscription(Bool, '/fold_arm/done', self.fold_arm_callback, 10)
        self.node_termination = False
        self.box_detection_terminated = False 
        self.alignment_to_face_terminated = False
        self.node_launched = False

        callback_group = ReentrantCallbackGroup()
        self.current_state = State.FOLDING_ARM
        
        # Setup executor for background tasks
        executor = rclpy.executors.MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        # Run state machine
        self.get_logger().info('Starting state machine...')
        self.state_machine_timer = self.create_timer(0.1, self.state_machine_step)
    
    def fold_arm_callback(self, msg):
        if msg.data:
            self.node_termination = True
    
    def nav_to_box_callback(self, msg):
        if msg.data:
            self.box_detection_terminated = True

    def align_to_box_face_callback(self, msg):
        if msg.data:
            self.alignment_to_face_terminated = True

    def run_node_subprocess(self, package, node, input = ""):
        """Run a ROS2 node as a subprocess."""
        # Format: ros2 run <package_name> <node_executable>
        cmd = ['ros2', 'run', package, node]
        if input:
            input_args = input.split()
            cmd.extend(input_args)
        process = subprocess.Popen(cmd)
        
        return process
    
    def run_node(self, package, node, args=None):
        """Run a ROS2 node and wait for it to finish."""
        cmd = ['ros2', 'run', package, node]

        if args:
            cmd.extend([str(arg) for arg in args])
            
        process = subprocess.run(cmd)

        return process
    
    def state_machine_step(self):
        """State machine for the navigation process."""
        #self.get_logger().info(f"Current state: {self.current_state.name}")

        if self.current_state == State.FOLDING_ARM:
            # Fold the arm
            if not self.node_launched:
                self.get_logger().info("Folding arm...")
                self.navigation_process = self.run_node('tiago_exam_arm', 'fold_arm')
                self.node_launched = True
                
            if self.node_termination:
                self.get_logger().info("Arm folded, transitioning to LOOKING_FOR_BOX")
                self.current_state = State.LOOKING_FOR_BOX
                self.node_launched = False
                
        elif self.current_state == State.LOOKING_FOR_BOX:
            # Look for the box
            if not self.node_launched:
                self.get_logger().info("Looking for box...")
                self.navigation_process = self.run_node_subprocess('tiago_exam_navigation', 'navigate_to_box')
                self.node_launched = True
                
            if self.box_detection_terminated:
                self.get_logger().info("Box found, transitioning to ALIGN_TO_BOX")
                self.current_state = State.ALIGN_TO_BOX
                self.node_launched = False
                
        elif self.current_state == State.ALIGN_TO_BOX:
            # Align to the detected box
            if not self.node_launched:
                self.get_logger().info("Aligning to box face...")
                self.align_process = self.run_node_subprocess('tiago_exam_navigation', 'align_to_box_face')
                self.node_launched = True
                
            if self.alignment_to_face_terminated:
                self.get_logger().info("Alignment complete, transitioning to DONE")
                self.current_state = State.DONE
                self.node_termination = False
                self.node_launched = False
                
        elif self.current_state == State.DONE:
            self.get_logger().info("STATE MACHINE COMPLETED")
            
            for i in range(3):
                self.done_publisher.publish(Bool(data=True))
                time.sleep(0.1)
            
            # Wait for a moment to ensure the message is sent
            time.sleep(1.0)
            
            #self.locking_target = self.run_node('tiago_exam_camera', 'target_locked')
            self.state_machine_timer.cancel()
        else:
            self.get_logger().error("Unknown state")

def main(args=None):
    rclpy.init(args=args)
    tiago_state_controller = StateMachineNavigation()
    rclpy.spin(tiago_state_controller)
    tiago_state_controller.get_logger().info("Shutting down state machine navigation")    
    # Cleanup
    tiago_state_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()