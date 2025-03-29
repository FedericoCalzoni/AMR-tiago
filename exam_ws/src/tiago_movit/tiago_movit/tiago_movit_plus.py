from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState

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

BASE_LINK_NAME = "base_link"
END_EFFECTOR_NAME = "arm_tool_link"
GROUP_NAME = "arm_torso"

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.joint_states = None

    def joint_state_callback(self, msg):
        self.joint_states = msg
        self.get_logger().info(f"Received joint state: {msg.position}")

    def wait_for_joint_states(self):
        while self.joint_states is None:
            self.get_logger().info("Waiting for joint states...")
            self.create_rate(1).sleep()


def main():
    rclpy.init()

    node = Node("example_tiago_joints_moveit2")

    joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK_NAME,
        end_effector_name=END_EFFECTOR_NAME,
        group_name=GROUP_NAME,
        callback_group=callback_group,
    )
    moveit2.planner_id = "RRTConnectkConfigDefault"

    # Aspetta che i joint states siano disponibili
    joint_state_listener = JointStateListener()
    joint_state_listener.wait_for_joint_states()

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 1.0

    # Move to pose
    node.get_logger().info(f"Moving to joints: {list(joint_positions)}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
