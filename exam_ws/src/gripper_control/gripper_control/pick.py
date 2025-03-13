import rclpy
from gripper_control.control_gripper import GripperController
from gripper_control.move_arm import ArmController
import time

def main(args=None):
    rclpy.init(args=args)
    gripper = GripperController()
    arm = ArmController()

    gripper.open_gripper()
    time.sleep(2)

    arm.move_arm_to_pose([0.0, -1.0, 0.5, 1.5, 0.0, 1.0, 0.0])  # Sopra il cubo
    time.sleep(2)

    gripper.close_gripper()
    time.sleep(2)

    arm.move_arm_to_pose([0.5, -0.5, 0.5, 1.0, 0.0, 1.0, 0.0])  # Trasporto
    time.sleep(2)

    arm.move_arm_to_pose([0.5, 0.0, 0.5, 1.5, 0.0, 1.0, 0.0])  # Sopra la scatola
    time.sleep(2)
    gripper.open_gripper()
    time.sleep(2)

    gripper.destroy_node()
    arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()