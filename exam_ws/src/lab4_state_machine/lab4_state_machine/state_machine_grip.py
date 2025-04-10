from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import GripperInterface
import time

# Gripper joint names (TIAGo)
JOINT_GRIPPER_NAMES = [
    "gripper_left_finger_joint",
    "gripper_right_finger_joint",
]

# Posizioni di apertura e chiusura
OPEN_GRIPPER_JOINT_POSITIONS = 0.04
CLOSED_GRIPPER_JOINT_POSITIONS = 0.01

class GripperStateMachine(Node):
    def __init__(self):
        super().__init__('gripper_state_machine_node')

        self.get_logger().info("⏳ Creo l'interfaccia gripper...")

        # Callback group per esecuzione parallela
        callback_group = ReentrantCallbackGroup()

        # Interfaccia al gripper
        self.gripper = GripperInterface(
            node=self,
            joint_names=["gripper_left_finger_joint", "gripper_right_finger_joint"],
            open_gripper_joint_positions=[0.04, 0.04],     # ✅ OK: lista
            closed_gripper_joint_positions=[0.01, 0.01]    # ✅ OK: lista
        )


        # Spin in background con MultiThreadedExecutor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self)
        thread = Thread(target=executor.spin, daemon=True)
        thread.start()

        # Breve attesa iniziale
        time.sleep(2)

    def run(self):
        for i in range(3):
            self.get_logger().info(f"🔁 Ciclo {i + 1}: Apertura...")
            self.gripper.move_to_position(OPEN_GRIPPER_JOINT_POSITIONS)
            self.gripper.wait_until_executed()
            time.sleep(1)

            self.get_logger().info(f"🔁 Ciclo {i + 1}: Chiusura...")
            self.gripper.move_to_position(CLOSED_GRIPPER_JOINT_POSITIONS)
            self.gripper.wait_until_executed()
            time.sleep(1)

        self.get_logger().info("✅ Gripper test completato!")

def main(args=None):
    rclpy.init(args=args)
    node = GripperStateMachine()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
