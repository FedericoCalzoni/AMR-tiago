import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import Bool
from time import sleep
from threading import Thread
import os

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')
                
        self.state = 'FOLD_ARM'
        
        self.node_launched = False
        self.mapping_done = False
        self.saving_done = False
        
        # Setup executor for background tasks
        executor = rclpy.executors.MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        # Run state machine
        self.get_logger().info('Starting mapping...')
        self.timer = self.create_timer(1.0, self.fsm_step)
    
    def run_node(self, package, node, args=None):
        """Run a ROS2 node and wait for it to finish."""
        cmd = ['ros2', 'run', package, node]

        if args:
            cmd.extend([str(arg) for arg in args])
            
        process = subprocess.run(cmd)

        return process
    
    def launch_and_monitor_exploration(self):
        """Launch explore_lite and monitor its logs for completion."""
        cmd = ['ros2', 'launch', 'explore_lite', 'explore.launch.py']
        self.get_logger().info("Launching explore_lite...")
        self.mapping_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )

        # Start a thread to monitor the logs
        def monitor_logs():
            for line in self.mapping_process.stdout:
                self.get_logger().info(f"[explore_lite] {line.strip()}")
                if "Exploration stopped" in line:
                    self.mapping_done = True
                    break

        self.mapping_done = False
        monitor_thread = Thread(target=monitor_logs, daemon=True)
        monitor_thread.start()
        
    def launch_and_monitor_map_saver(self):
        """Launch map_saver_cli and monitor its logs for completion."""
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', os.path.expanduser('~/AMR-tiago/maps/map')
        ]
        self.get_logger().info("Launching map_saver_cli...")
        self.saver_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )

        # Start a thread to monitor logs
        def monitor_logs():
            for line in self.saver_process.stdout:
                self.get_logger().info(f"[map_saver] {line.strip()}")
                if "Map saved successfully" in line:
                    self.get_logger().info("Map has been saved successfully.")
                    self.saving_done = True
                    break

        self.saving_done = False
        monitor_thread = Thread(target=monitor_logs, daemon=True)
        monitor_thread.start()

        
    def fsm_step(self):
        """State machine for mapping"""
        if self.state == 'FOLD_ARM':
            self.get_logger().info("FOLD_ARM: Folding arm")
            self.run_node('tiago_exam_arm', 'fold_arm')
            self.state = 'MAPPING'
            
        if self.state == 'MAPPING':
            if not self.node_launched:
                self.get_logger().info("MAPPING: Starting explore lite")
                self.launch_and_monitor_exploration()
                self.node_launched = True
                
            if self.mapping_done:
                self.get_logger().info(f"MAPPING: Completed")
                self.mapping_process.terminate()
                self.state = 'SAVE_MAP'
                self.node_launched = False
            
        elif self.state == 'SAVE_MAP':
            if not self.node_launched:
                self.get_logger().info("SAVE_MAP: Save the map")
                self.launch_and_monitor_map_saver()
                self.node_launched = True
                
            if self.saving_done:
                self.get_logger().info("SAVE_MAP: completed")
                self.saver_process.terminate()
                self.state = 'RETURN_HOME'
                self.node_launched = False
            
        elif self.state == 'RETURN_HOME':
            self.get_logger().info("RETURN_HOME: Finishing the task")
            self.run_node("tiago_exam_navigation", "navigate_to_pose", args=['--goal', '0.0', '0.0', '0.0'])
            self.state = 'DONE'
            
        elif self.state == 'DONE':
            self.get_logger().info("Task sequence completed")
            self.timer.cancel()  # Stop the timer
            
        else:
            raise ValueError(f"Unrecognized state: {self.state}")
  
            

def main(args=None):
    rclpy.init(args=args)
    task_manager_node = TaskManager()
    
    # Use a separate thread to spin the node
    spin_thread = Thread(target=rclpy.spin, args=(task_manager_node,))
    spin_thread.start()
    
    try:
        # Wait for the spin thread to finish (it won't unless we shutdown)
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
