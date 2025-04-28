import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import Bool
from threading import Thread
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # State flags
        self.node_launched = False
        self.navigation_done = False
        self.move_arm_done = False
        self.node_completed = False

        # Initial state
        self.state = 'MOVE_TO_PICK_582'

        # Callback group and subscriptions
        callback_group = ReentrantCallbackGroup()
        self.create_subscription(Bool, '/state_machine_navigation/done', self.navigation_callback, 10, callback_group=callback_group)
        self.create_subscription(Bool, '/move_arm/done', self.move_arm_callback, 10, callback_group=callback_group)

        # Publisher to start navigation FSM
        self.start_nav_pub = self.create_publisher(Bool, '/start_navigation_state_machine', 10)

        # Executor
        executor = MultiThreadedExecutor(4)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # FSM timer
        self.get_logger().info('Starting state machine...')
        self.timer = self.create_timer(1.0, self.fsm_step)

    def navigation_callback(self, msg):
        if msg.data:
            self.navigation_done = True
            self.node_completed = True

    def move_arm_callback(self, msg):
        if msg.data:
            self.move_arm_done = True
            self.node_completed = True

    def run_node(self, package, node, args=None):
        cmd = ['ros2', 'run', package, node]
        if args:
            cmd.extend([str(arg) for arg in args])
        subprocess.Popen(cmd)

    def start_navigation_fsm(self):
        time.sleep(1.0)  # Wait for the node to spin up
        self.start_nav_pub.publish(Bool(data=True))
        self.get_logger().info("Sent start signal to navigation state machine")

    def reset_flags(self):
        self.navigation_done = False
        self.move_arm_done = False
        self.node_launched = False
        self.node_completed = False

    def fsm_step(self):
        if self.state == 'MOVE_TO_PICK_582':
            if not self.node_launched:
                self.get_logger().info("MOVE_TO_PICK_582: Starting navigation to pick box")
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
                self.start_navigation_fsm()
                self.node_launched = True

            elif self.navigation_done:
                self.get_logger().info("MOVE_TO_PICK_582: Completed")
                self.state = 'PICK_CUBE_582'
                self.reset_flags()

        elif self.state == 'PICK_CUBE_582':
            if not self.node_launched:
                self.get_logger().info("PICK_CUBE_582: Picking the cube")
                self.run_node("tiago_exam_camera", "target_locked", ["582"])
                self.run_node("tiago_exam_arm", "2_aruco_grasp_pose_broadcaster")
                self.run_node("tiago_exam_arm", "3_move_arm", ['--action', 'PICK582'])
                self.node_launched = True

            elif self.move_arm_done:
                self.get_logger().info("PICK_CUBE_582: Arm movement done")
                self.state = 'TRANSFER_582'
                self.reset_flags()

        elif self.state == 'TRANSFER_582':
            if not self.node_launched:
                self.get_logger().info("TRANSFER_582: Moving to placement area")
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
                self.start_navigation_fsm()
                self.node_launched = True

            elif self.navigation_done:
                self.get_logger().info("TRANSFER_582: Completed")
                self.state = 'PLACE_CUBE_582'
                self.reset_flags()

        elif self.state == 'PLACE_CUBE_582':
            if not self.node_launched:
                self.get_logger().info("PLACE_CUBE_582: Placing the cube")
                self.run_node("tiago_exam_arm", "3_move_arm", ['--action', 'PLACE582'])
                self.node_launched = True

            elif self.move_arm_done:
                self.get_logger().info("PLACE_CUBE_582: Done")
                self.state = 'MOVE_TO_PICK_63'
                self.reset_flags()

        elif self.state == 'MOVE_TO_PICK_63':
            if not self.node_launched:
                self.get_logger().info("MOVE_TO_PICK_63: Moving to second cube")
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
                self.start_navigation_fsm()
                self.node_launched = True

            elif self.navigation_done:
                self.get_logger().info("MOVE_TO_PICK_63: Completed")
                self.state = 'PICK_CUBE_63'
                self.reset_flags()

        elif self.state == 'PICK_CUBE_63':
            if not self.node_launched:
                self.get_logger().info("PICK_CUBE_63: Picking second cube")
                self.run_node("tiago_exam_arm", "3_move_arm", ['--action', 'PICK63'])
                self.node_launched = True

            elif self.move_arm_done:
                self.get_logger().info("PICK_CUBE_63: Done")
                self.state = 'TRANSFER_63'
                self.reset_flags()

        elif self.state == 'TRANSFER_63':
            if not self.node_launched:
                self.get_logger().info("TRANSFER_63: Moving to place second cube")
                self.run_node("tiago_exam_navigation", "state_machine_navigation")
                self.start_navigation_fsm()
                self.node_launched = True

            elif self.navigation_done:
                self.get_logger().info("TRANSFER_63: Completed")
                self.state = 'PLACE_CUBE_63'
                self.reset_flags()

        elif self.state == 'PLACE_CUBE_63':
            if not self.node_launched:
                self.get_logger().info("PLACE_CUBE_63: Placing second cube")
                self.run_node("tiago_exam_arm", "3_move_arm", ['--action', 'PLACE63'])
                self.node_launched = True

            elif self.move_arm_done:
                self.get_logger().info("PLACE_CUBE_63: Done")
                self.state = 'RETURN_HOME'
                self.reset_flags()

        elif self.state == 'RETURN_HOME':
            if not self.node_launched:
                self.get_logger().info("RETURN_HOME: Navigating to home position")
                self.run_node("tiago_exam_navigation", "navigate_to_pose", ['--goal', '0.0', '-1.0', '0.0'])
                self.node_launched = True

            elif self.navigation_done:
                self.get_logger().info("RETURN_HOME: Done")
                self.state = 'DONE'
                self.reset_flags()

        elif self.state == 'DONE':
            self.get_logger().info("All tasks completed ✅")
            self.timer.cancel()

        else:
            raise ValueError(f"Unrecognized state: {self.state}")

def main(args=None):
    rclpy.init(args=args)
    task_manager_node = TaskManager()

    try:
        rclpy.spin(task_manager_node)
    except KeyboardInterrupt:
        pass
    finally:
        task_manager_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
