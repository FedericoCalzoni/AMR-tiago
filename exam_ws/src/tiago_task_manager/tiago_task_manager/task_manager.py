import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import Bool
from time import sleep
from threading import Thread
from rclpy.callback_groups import ReentrantCallbackGroup

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        
        self.node_launched = False
        
        # Subscription to node termination topics
        self.create_subscription(Bool, '/state_machine_navigation/done', self.navigation_callback, 10)
        self.done_publisher = self.create_publisher(Bool, '/state_machine_navigation/done', 10)
        self.create_subscription(Bool, '/move_arm/done', self.move_arm_callback, 10)
        self.navigation_done = False
        self.move_arm_done = False
        
        callback_group = ReentrantCallbackGroup()
        
        self.state = 'MOVE_TO_PICK_63'
        
        # Setup executor for background tasks
        executor = rclpy.executors.MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        
        # Run state machine
        self.get_logger().info('Starting state machine...')
        self.timer = self.create_timer(1.0, self.fsm_step)
        
    def navigation_callback(self, msg):
        if msg.data == True:
            self.navigation_done = True
        else:
            self.navigation_done = False
            
    def move_arm_callback(self, msg):
        if msg.data == True:
            self.move_arm_done = True
            
    def run_node_subprocess(self, package, node, args=None):
        cmd = ['ros2', 'run', package, node]

        if args:
            cmd.extend([str(arg) for arg in args])

        process = subprocess.Popen(cmd)

        return process
    
    def run_node(self, package, node, args=None):
        cmd = ['ros2', 'run', package, node]

        if args:
            cmd.extend([str(arg) for arg in args])
            
        process = subprocess.run(cmd)

        return process
    
    def fsm_step(self):
        if self.state == 'MOVE_TO_PICK_63':
            if not self.node_launched:
                self.get_logger().info("MOVE_TO_PICK_63: Starting navigation to pick box")
                self.run_node_subprocess("tiago_exam_navigation", "state_machine_navigation")
                self.node_launched = True
                
            if self.navigation_done:
                self.get_logger().info(f"MOVE_TO_PICK_63: Completed")
                self.state = 'PICK_CUBE_63'
                self.node_launched = False
                self.navigation_done = False
            
        elif self.state == 'PICK_CUBE_63':
            if not self.node_launched:
                self.get_logger().info("PICK_CUBE_63: Move the arm to pick the cube")
                self.move_arm_done = False
                self.node_launched = True
                self.run_node_subprocess("tiago_exam_camera", "target_locked", ["63"])
                self.run_node_subprocess("tiago_exam_arm", "aruco_grasp_pose_broadcaster")
                self.run_node_subprocess("tiago_exam_arm", "move_arm", args=['--action', 'PICK63'])
                
            if self.move_arm_done:
                self.get_logger().info("Arm movement completed")
                self.run_node("tiago_exam_navigation", "move_head_to_pose", args=['0.0', '-0.2'])
                self.state = 'MOVE_TO_PLACE_63'
                self.node_launched = False
        
        elif self.state == 'MOVE_TO_PLACE_63':
            if not self.node_launched:
                self.get_logger().info("MOVE_TO_PLACE_63: Move tiago to place box")
                self.run_node_subprocess("tiago_exam_navigation", "state_machine_navigation")
                self.node_launched = True
                self.navigation_done = False
            
            if self.navigation_done:
                self.get_logger().info("MOVE_TO_PLACE_63: Completed")
                self.state = 'PLACE_CUBE_63'
                self.node_launched = False
            
        elif self.state == 'PLACE_CUBE_63':
            if not self.node_launched:
                self.get_logger().info("PLACE_CUBE_63: Move the arm to place the cube")
                self.node_launched = True
                self.move_arm_done = False
                self.run_node_subprocess("tiago_exam_arm", "move_arm", args=['--action', 'PLACE63'])
                
            if self.move_arm_done:
                self.get_logger().info("Arm movement completed")
                self.state = 'MOVE_TO_PICK_582'
                self.node_launched = False
        
        elif self.state == 'MOVE_TO_PICK_582':
            if not self.node_launched:
                self.get_logger().info("MOVE_TO_PICK_582: Starting navigation to pick box")
                self.run_node_subprocess("tiago_exam_navigation", "state_machine_navigation")
                self.node_launched = True
                self.navigation_done = False
                
            if self.navigation_done:
                self.get_logger().info(f"MOVE_TO_PICK_582: Completed")
                self.state = 'PICK_CUBE_582'
                self.node_launched = False
            
        elif self.state == 'PICK_CUBE_582':
            if not self.node_launched:
                self.get_logger().info("PICK_CUBE_582: Move the arm to pick the cube")
                self.node_launched = True
                self.move_arm_done = False
                self.run_node_subprocess("tiago_exam_camera", "target_locked", ["582"])
                self.run_node_subprocess("tiago_exam_arm", "aruco_grasp_pose_broadcaster")
                self.run_node_subprocess("tiago_exam_arm", "move_arm", args=['--action', 'PICK582'])
                
            if self.move_arm_done:
                self.get_logger().info("PICK_CUBE_582: Completed")
                self.run_node("tiago_exam_navigation", "move_head_to_pose", args=['0.0', '-0.2'])
                self.state = 'MOVE_TO_PLACE_582'
                self.node_launched = False
            
        elif self.state == 'MOVE_TO_PLACE_582':
            if not self.node_launched:
                self.get_logger().info("MOVE_TO_PLACE_582: Move tiago to place box")
                self.run_node_subprocess("tiago_exam_navigation", "state_machine_navigation")
                self.node_launched = True
                self.navigation_done = False
            
            if self.navigation_done:
                self.get_logger().info("MOVE_TO_PLACE_582: Completed")
                self.state = 'PLACE_CUBE_582'
                self.node_launched = False
            
        elif self.state == 'PLACE_CUBE_582':
            if not self.node_launched:
                self.get_logger().info("PLACE_CUBE_582: Move the arm to place the cube")
                self.node_launched = True
                self.move_arm_done = False
                self.run_node_subprocess("tiago_exam_arm", "move_arm", args=['--action', 'PLACE582'])
                
            if self.move_arm_done:
                self.get_logger().info("PLACE_CUBE_582: Completed")
                self.state = 'RETURN_HOME'
                self.node_launched = False
            
        elif self.state == 'RETURN_HOME':
            self.get_logger().info("RETURN_HOME: Finishing the task")
            self.run_node('tiago_exam_arm', 'fold_arm')
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
