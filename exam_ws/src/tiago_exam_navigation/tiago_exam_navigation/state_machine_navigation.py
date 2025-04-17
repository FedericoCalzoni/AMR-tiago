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
        if process.poll() is None:  # Check if process is still running
            try:
                node_info = f"PID: {process.pid}"
                self.get_logger().info(f"Stopping node {node_info}")
                
                # First attempt: Use ROS2 CLI to get a clean shutdown
                try:
                    # Try to find node name from PID
                    node_name = self._get_node_name_from_pid(process.pid)
                    if node_name:
                        self.get_logger().info(f"Attempting to shut down node {node_name} via ROS2 API")
                        # Use ros2 node info to verify it exists
                        subprocess.run(f"ros2 node info {node_name}", 
                                    shell=True, timeout=2, 
                                    stdout=subprocess.PIPE, 
                                    stderr=subprocess.PIPE)
                except Exception:
                    # If can't get node name or ros2 command fails, proceed with signal-based approach
                    node_name = None
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    # Try direct process group termination
                    self.get_logger().info(f"Node {node_info} not responding to SIGINT, trying SIGTERM")
                    try:
                        # Try to terminate the entire process group
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    except Exception:
                        # Fall back to regular terminate if process group approach fails
                        process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        # Last resort: SIGKILL
                        self.get_logger().info(f"Node {node_info} not responding to SIGTERM, trying SIGKILL")
                        try:
                            # Try to kill the entire process group
                            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        except Exception:
                            # Fall back to regular kill if process group approach fails
                            process.kill()
                        try:
                            process.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            # If still can't kill, call external tools
                            self._force_kill_process(process.pid)
                if process.poll() is None:
                    self.get_logger().error(f"Failed to kill node {node_info}")
                else:
                    self.get_logger().info(f"Node {node_info} terminated with exit code {process.returncode}")
                    
            except Exception as e:
                self.get_logger().error(f"Error while stopping node: {e}")
        else:
            self.get_logger().info(f"Node (PID: {process.pid}) was already stopped")

    def _get_node_name_from_pid(self, pid):
        try:
            # List all nodes with their PIDs and grep for our PID
            cmd = f"ps -ef | grep {pid} | grep -v grep"
            output = subprocess.check_output(cmd, shell=True, text=True)
            
            # Parse output to find potential node name
            lines = output.strip().split('\n')
            for line in lines:
                if f"{pid}" in line and "ros2 run" in line:
                    parts = line.split()
                    # Find the node executable name
                    for i, part in enumerate(parts):
                        if part == "run" and i+2 < len(parts):
                            return parts[i+2]  # Package name + node name
            return None
        except Exception:
            return None

    def _force_kill_process(self, pid):
        """Force kill a process when all else fails"""
        try:
            # Try different methods in sequence
            methods = [
                # 1. Standard Linux kill with force flag
                f"kill -9 {pid}",
                
                # 2. Find and kill child processes first
                f"pkill -TERM -P {pid}",
                
                # 3. Find process with ptree and kill entire hierarchy
                f"pstree -p {pid} | grep -o '([0-9]\\+)' | grep -o '[0-9]\\+' | xargs kill -9"
            ]
            for i, method in enumerate(methods):
                try:
                    self.get_logger().info(f"Force kill attempt {i+1} on PID {pid}")
                    subprocess.run(method, shell=True, timeout=2)
                    # Check if process is gone
                    time.sleep(0.5)
                    try:
                        os.kill(pid, 0)  # This will raise an error if process is gone
                        # If we get here, process still exists
                        continue
                    except ProcessLookupError:
                        # Process is gone
                        self.get_logger().info(f"Successfully killed PID {pid} with method {i+1}")
                        return True
                except Exception as e:
                    self.get_logger().debug(f"Kill method {i+1} failed: {e}")
            try:
                os.kill(pid, 0)
                self.get_logger().error(f"Process {pid} could not be killed by any method")
                return False
            except ProcessLookupError:
                # Process is gone
                return True
        except Exception as e:
            self.get_logger().error(f"Error in force kill: {e}")
            return False
    
    def state_machine_step(self):
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
            self.get_logger().info("STATE MACHINE COMPLETED")
            #self.locking_target = self.run_node('tiago_exam_camera', 'target_locked')
            self.stop_node(self.navigation_process)
            self.stop_node(self.align_process)
            self.state_machine_timer.cancel()
            
        else:
            self.get_logger().error("Unknown state")

def main(args=None):
    rclpy.init(args=args)
    tiago_state_controller = StateMachineNavigation()
    rclpy.spin(tiago_state_controller)
    tiago_state_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()