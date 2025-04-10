import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2

# Definisci le giunture del braccio
JOINT_ARM_NAMES = [
    "torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint",
    "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint", "arm_tool_joint"
]

class ArmMovementNode(Node):
    def __init__(self):
        super().__init__('arm_movement_node')

        # Inizializza MoveIt2 per il braccio
        self.arm = MoveIt2(
            node=self,
            joint_names=JOINT_ARM_NAMES,
            group_name="arm_torso"
        )

        self.arm.planner_id = "RRTConnectkConfigDefault"  # Puoi scegliere un altro planner se preferisci

    def move_arm_to_pose(self, position, orientation):
        self.arm.move_to_pose(position=position, quat_xyzw=orientation)
        self.arm.wait_until_executed()

def main(args=None):
    rclpy.init(args=args)
    node = ArmMovementNode()

    # Definisci la posizione di destinazione
    target_position = [0.5, 0.0, 0.8]
    target_orientation = [0.0, 0.0, 0.0, 1.0]  # orientamento neutro (quaternion)

    node.move_arm_to_pose(target_position, target_orientation)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
