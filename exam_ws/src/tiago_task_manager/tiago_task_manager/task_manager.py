import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import Bool
import time
from threading import Thread

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        self.state = 'MOVE_TO_PICK_582'
        
        self.node_completed = True
        self.launched = False
        
        self.create_subscription(Bool, '/state_machine_navigation/done', self.completed_callback, 10)
        
        self.timer = self.create_timer(1.0, self.fsm_step)
        
        self.get_logger().info("Starting the Finite State Machine (FSM)")

    def fsm_step(self):
        if self.state == 'MOVE_TO_PICK_582':
            if not self.launched and not self.node_completed:
                self.get_logger().info("MOVE_TO_PICK: Starting navigation to pick box")
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
                self.launched = True
                self.node_completed = False
            elif self.node_completed:
                self.get_logger().info(f"Navigation completed, node_completed: {self.node_completed}")
                self.launched = False  # Reset for next navigation
                self.node_completed = False
                self.state = 'PICK_CUBE_582'
            else:
                self.get_logger().info("Waiting for navigation to finish...")
            
        elif self.state == 'PICK_CUBE_582':
            self.get_logger().info("PICK_CUBE_582: Move the arm to pick the cube")
            self.run_node("tiago_exam_arm", "3_move_arm", '--action PICK582')
            self.state = 'TRANSFER_582'
        
        elif self.state == 'TRANSFER_582':
            self.get_logger().info("TRANSFER_582: Move tiago to place box")
            self.launched = False  # Reset for new navigation
            self.node_completed = False
            self.run_node("tiago_exam_navigation", "state_machine_navigation")
            self.state = 'WAIT_TRANSFER_582'
            
        elif self.state == 'WAIT_TRANSFER_582':
            if self.node_completed:
                self.get_logger().info("Transfer completed")
                self.launched = False
                self.state = 'PLACE_CUBE_582'
            
        elif self.state == 'PLACE_CUBE_582':
            self.get_logger().info("PLACE_CUBE_582: Move the arm to place the cube")
            self.run_node("tiago_exam_arm", "3_move_arm", '--action PLACE582')
            self.state = 'MOVE_TO_PICK_63'
        
        elif self.state == 'MOVE_TO_PICK_63':
            if not self.launched:
                self.get_logger().info("MOVE_TO_PICK_63: Starting navigation to pick box")
                self.launched = True
                self.node_completed = False
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
            elif self.node_completed:
                self.get_logger().info("Navigation to pick 63 completed")
                self.launched = False
                self.state = 'PICK_CUBE_63'
            
        elif self.state == 'PICK_CUBE_63':
            self.get_logger().info("PICK_CUBE_63: Move the arm to pick the cube")
            self.run_node("tiago_exam_arm", "3_move_arm", '--action PICK63')
            self.state = 'TRANSFER_63'
            
        elif self.state == 'TRANSFER_63':
            if not self.launched:
                self.get_logger().info("TRANSFER_63: Move tiago to place box")
                self.launched = True
                self.node_completed = False
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
            elif self.node_completed:
                self.get_logger().info("Transfer 63 completed")
                self.launched = False
                self.state = 'PLACE_CUBE_63'
            
        elif self.state == 'PLACE_CUBE_63':
            self.get_logger().info("PLACE_CUBE_63: Move the arm to place the cube")
            self.run_node("tiago_exam_arm", "3_move_arm", '--action PLACE63')
            self.state = 'RETURN_HOME'
            
        elif self.state == 'RETURN_HOME':
            self.get_logger().info("RETURN_HOME: Finishing the task")
            self.run_node("tiago_exam_navigation", "navigate_to_pose", '--goal 0.0 -1.0 0.0')
            self.state = 'DONE'
            
        elif self.state == 'DONE':
            self.get_logger().info("Task sequence completed")
            self.timer.cancel()  # Stop the timer
            
        else:
            raise ValueError(f"Unrecognized state: {self.state}")
  
    def completed_callback(self, msg):
        if msg.data == True:
            self.node_completed = True
            self.get_logger().info("Callback: Node completed.")
            
    def run_node(self, package, node, args=None):
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
