from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2


##############################
# Tiago Parameters

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

##############################


def main():
    rclpy.init()

    # Crea il nodo per questo esempio
    node = Node("example_tiago_joints_moveit2")

    joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Crea un gruppo di callback che permette l'esecuzione di callback in parallelo senza restrizioni
    callback_group = ReentrantCallbackGroup()

    # Crea l'interfaccia MoveIt2
    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK_NAME,
        end_effector_name=END_EFFECTOR_NAME,
        group_name=GROUP_NAME,
        callback_group=callback_group,
    )
    moveit2.planner_id = "RRTConnectkConfigDefault"

    # Esegui il nodo in un thread di background e attendi un po' per l'inizializzazione
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Imposta la velocità e l'accelerazione massima dei giunti (percentuale del massimo)
    moveit2.max_velocity = 1.0
    moveit2.max_acceleration = 1.0

    # Muovi verso la posa
    node.get_logger().info(f"Moving to joints: {list(joint_positions)}")
    try:
        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()
        node.get_logger().info("Movement completed successfully!")
    except Exception as e:
        node.get_logger().error(f"Error during movement: {e}")
    
    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()