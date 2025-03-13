import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from time import sleep

# Parametri per il gripper
JOINT_NAMES = [
    "gripper_left_finger_joint",  # Giunto sinistro
    "gripper_right_finger_joint", # Giunto destro
]

OPEN_GRIPPER_JOINT_POSITIONS = [0.04, 0.04]  # Posizione aperta
CLOSED_GRIPPER_JOINT_POSITIONS = [0.0, 0.0]  # Posizione chiusa

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Crea i publisher per i giunti del gripper
        self.gripper_left_pub = self.create_publisher(Float64, '/gripper_left_finger_joint/command', 10)
        self.gripper_right_pub = self.create_publisher(Float64, '/gripper_right_finger_joint/command', 10)

    def close_gripper(self):
        # Chiude il gripper
        self.get_logger().info("Chiudendo il gripper...")
        left_msg = Float64()
        right_msg = Float64()

        left_msg.data = CLOSED_GRIPPER_JOINT_POSITIONS[0]
        right_msg.data = CLOSED_GRIPPER_JOINT_POSITIONS[1]

        # Pubblica i comandi per entrambi i giunti
        self.gripper_left_pub.publish(left_msg)
        self.gripper_right_pub.publish(right_msg)

    def open_gripper(self):
        # Apre il gripper
        self.get_logger().info("Aprendo il gripper...")
        left_msg = Float64()
        right_msg = Float64()

        left_msg.data = OPEN_GRIPPER_JOINT_POSITIONS[0]
        right_msg.data = OPEN_GRIPPER_JOINT_POSITIONS[1]

        # Pubblica i comandi per entrambi i giunti
        self.gripper_left_pub.publish(left_msg)
        self.gripper_right_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)

    gripper_controller = GripperController()

    # Test: apri e chiudi il gripper
    gripper_controller.open_gripper()
    sleep(2)  # Attendi 2 secondi
    gripper_controller.close_gripper()
    sleep(2)  # Attendi 2 secondi

    # Interrompi il nodo dopo il test
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
import rclpy
from rclpy.node import Node
from control_msgs.msg import GripperCommand
from std_msgs.msg import Bool

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        # Crea un publisher per il gripper
        self.gripper_pub = self.create_publisher(GripperCommand, '/gripper_controller/command', 10)

    def close_gripper(self):
        # Invia un comando per chiudere il gripper
        gripper_msg = GripperCommand()
        gripper_msg.position = 0.0  # Impostare la posizione desiderata per chiudere
        gripper_msg.max_effort = 100.0  # Impostare lo sforzo massimo
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info("Gripper chiuso!")

    def open_gripper(self):
        # Invia un comando per aprire il gripper
        gripper_msg = GripperCommand()
        gripper_msg.position = 1.0  # Impostare la posizione desiderata per aprire
        gripper_msg.max_effort = 100.0  # Impostare lo sforzo massimo
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info("Gripper aperto!")

def main(args=None):
    rclpy.init(args=args)
    gripper_controller = GripperController()

    # Test: chiudi il gripper, quindi aprilo
    gripper_controller.open_gripper()
    rclpy.spin_once(gripper_controller, timeout_sec=10.0)
    gripper_controller.close_gripper()
    rclpy.spin_once(gripper_controller, timeout_sec=10.0)
    gripper_controller.open_gripper()

    rclpy.spin(gripper_controller)

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from control_msgs.msg import GripperCommand

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        # Crea un publisher per il gripper
        self.gripper_pub = self.create_publisher(GripperCommand, '/gripper_controller/command', 10)

    def close_gripper(self):
        # Invia un comando per chiudere il gripper
        gripper_msg = GripperCommand()
        gripper_msg.position = 0.0  # Impostare la posizione desiderata per chiudere
        gripper_msg.max_effort = 100.0  # Impostare lo sforzo massimo
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info("Gripper chiuso!")
        
        # Aggiungi un breve ritardo per assicurarti che il comando venga eseguito
        #rclpy.sleep(2.0)  # Pausa di 2 secondi

    def open_gripper(self):
        # Invia un comando per aprire il gripper
        gripper_msg = GripperCommand()
        gripper_msg.position = 1.0  # Impostare la posizione desiderata per aprire
        gripper_msg.max_effort = 100.0  # Impostare lo sforzo massimo
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info("Gripper aperto!")
        
        # Aggiungi un breve ritardo per assicurarti che il comando venga eseguito
        #rclpy.sleep(2.0)  # Pausa di 2 secondi

def main(args=None):
    rclpy.init(args=args)
    gripper_controller = GripperController()

    # Test: chiudi il gripper, quindi aprilo
    gripper_controller.open_gripper()
    rclpy.spin_once(gripper_controller, timeout_sec=10.0)
    
    # Dopo aver aperto il gripper, chiudilo
    gripper_controller.close_gripper()
    rclpy.spin_once(gripper_controller, timeout_sec=10.0)
    
    # Riapri il gripper
    gripper_controller.open_gripper()
    rclpy.spin_once(gripper_controller, timeout_sec=10.0)

    rclpy.spin(gripper_controller)

if __name__ == '__main__':
    main()
"""