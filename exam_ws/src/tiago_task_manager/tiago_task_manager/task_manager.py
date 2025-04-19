import rclpy
from rclpy.node import Node
import subprocess
import time

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')
        self.state = 0
        self.get_logger().info("Starting the Finite State Machine (FSM)")

    def run_fsm(self):
        while rclpy.ok():
            # State 0: Move tiago to pick box
            if self.state ==  0:
                self.get_logger().info("State 0: Starting navigation to pick box")
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
                self.state = 1
                """
            # State 1: Move the arm to pick the cube
            elif self.state == 1:
                self.get_logger().info("Stato 1: Move the arm to pick the cube")
                self.run_node("tiago_exam_arm", "4_pick_and_place")
                self.state = 2
            # State 2: Move tiago to place box
            elif self.state == 2:
                self.get_logger().info("State 2: Move tiago to place box")
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
                self.state = 3
            # State 3: Move the arm to place the cube
            elif self.state == 3:
                self.get_logger().info("State 3: Move the arm to place the cube")
                self.run_node("tiago_exam_arm", "4_pick_and_place")
                self.state = 4
            # State 4: Return to the pick box
            elif self.state == 4:
                self.get_logger().info("State 4: Return to the pick box")
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
                self.state = 5
            # State 5: Move the arm to pick the cube
            elif self.state == 5:
                self.get_logger().info("State 5: Move the arm to pick the cube")
                self.run_node("tiago_exam_arm", "4_pick_and_place")
                self.state = 6
            # State 6: Move tiago to place box
            elif self.state == 6:
                self.get_logger().info("State 6: Move tiago to place box")
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
                self.state = 7
            # State 7: Move the arm to place the cube
            elif self.state == 7:
                self.get_logger().info("State 7: Move the arm to place the cube")
                self.run_node("tiago_exam_arm", "4_pick_and_place")
                self.state = 8

            # State 8: Go in the home position and spin around yourself
            elif self.state == 8:
                self.get_logger().info("State 8: Finisched the task, going in the home position and spinning around")
                """

            # Add additional states as needed
            else:
                self.get_logger().warn("Unrecognized state. Exiting FSM.")
                break

    def run_node(self, pkg, executable):
        """Runs a ROS 2 node using ros2 run and waits for it to complete."""
        self.get_logger().info(f"Launching node: {pkg}/{executable}")
        
        # Run the node synchronously and wait for it to exit
        process = subprocess.run(["ros2", "run", pkg, executable])
        
        # Check exit status and log accordingly
        if process.returncode != 0:
            self.get_logger().error(f"Error occurred in {pkg}/{executable}")
        else:
            self.get_logger().info(f"Node {pkg}/{executable} completed successfully.")

def main(args=None):
    rclpy.init(args=args)
    task_manager_node = TaskManager()
    task_manager_node.run_fsm()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
