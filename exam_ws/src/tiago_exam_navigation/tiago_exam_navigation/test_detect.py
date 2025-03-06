import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class BlueBlockFollower(Node):
    def __init__(self):
        super().__init__('blue_block_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.head_pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)  # Publisher per il controllo della testa

    def image_callback(self, msg):
        # Converti l'immagine ROS in OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Converti l'immagine in spazio colore HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Definisci l'intervallo di colore per il celeste
        lower_blue = np.array([90, 50, 50])   
        upper_blue = np.array([130, 255, 255]) 

        # Crea una maschera per il colore blu
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Trova i contorni dell'oggetto blu
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cmd = Twist()  # Crea un messaggio Twist

        if contours:
            # Trova il contorno più grande
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Controlla se il contorno è abbastanza grande
            min_contour_area = 6000  # Area minima del contorno per considerarlo valido (da regolare)
            if cv2.contourArea(largest_contour) > min_contour_area:
                # Trova il centro del blocco
                object_center_x = x + w // 2
                frame_center_x = cv_image.shape[1] // 2  # Centro dell'immagine

                # Disegna un rettangolo attorno all'oggetto rilevato
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Ferma il robot
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

                # Regola la testa per trovare una box di dimensioni specifiche
                desired_box_width = 0.5  # Larghezza desiderata della box (da regolare)
                desired_box_height = 0.5  # Altezza desiderata della box (da regolare)

                # Calcola la posizione della testa in base alle dimensioni della box
                head_cmd = JointTrajectory()
                head_cmd.joint_names = ["head_1_joint", "head_2_joint"]
                point = JointTrajectoryPoint()
                point.positions = [0.0, -0.5 * ((w * h) / (desired_box_width * desired_box_height))]  # Regola la posizione della testa
                point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
                head_cmd.points = [point]
                self.head_pub.publish(head_cmd)

                self.get_logger().info(f"Blocco rilevato: x={object_center_x}, larghezza: {w}, altezza: {h}")
                self.get_logger().info(f"Comando testa pubblicato: {head_cmd}")
            else:
                # Se il contorno non è abbastanza grande, ferma il robot
                cmd.linear.x = 0.0
                cmd.angular.z = 0.3  # Prova a girarsi per cercare il blocco

        else:
            # Se non vede nulla, ferma il robot
            cmd.linear.x = 0.0
            cmd.angular.z = -0.3  # Prova a girarsi per cercare il blocco

        # Pubblica il comando di velocità
        self.cmd_vel_pub.publish(cmd)

        # Mostra l'immagine per debug
        cv2.imshow("Blue Block Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BlueBlockFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()