import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
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
        self.head_pub = self.create_publisher(JointTrajectory, '/head_controller/command', 10)  # Publisher per il controllo della testa
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)
        self.rotating = True
        self.box_detected = False
        self.box_position = None
        self.threshold_distance = 0.2  # Distanza di sicurezza dalla box (da regolare)

    def image_callback(self, msg):
        if self.box_detected:
            return

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
            min_contour_area = 9300  # Area minima del contorno per considerarlo valido (da regolare)
            if cv2.contourArea(largest_contour) > min_contour_area:
                # Trova il centro del blocco
                object_center_x = x + w // 2
                frame_center_x = cv_image.shape[1] // 2  # Centro dell'immagine

                # Disegna un rettangolo attorno all'oggetto rilevato
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calcola l'orientamento della box
                rect = cv2.minAreaRect(largest_contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                angle = rect[2]

                # Se l'angolo è negativo, aggiungi 90 gradi
                if angle < -45:
                    angle += 90

                # Controllo della rotazione: se il blocco è a destra o sinistra dell'immagine
                error_x = frame_center_x - object_center_x  # Differenza dal centro dell'immagine
                angular_speed = -0.005 * error_x  # Rotazione più veloce se l'errore è grande

                # Limita la velocità angolare per evitare rotazioni troppo rapide
                max_angular_speed = 0.75
                angular_speed = max(min(angular_speed, max_angular_speed), -max_angular_speed)

                # Controllo della velocità lineare: rallenta se l'oggetto è vicino
                object_size = w * h  # Area del blocco rilevato
                max_size = 30000  # Soglia per fermarsi (da regolare)

                if object_size < max_size:
                    cmd.linear.x = 0.75  # Vai avanti
                    cmd.angular.z = 0.0
                else:
                    cmd.linear.x = 0.0  # Fermati se sei vicino

                #cmd.angular.z = angular_speed  # Ruota per centrare il blocco
                # Abbassa la testa gradualmente mentre il robot si avvicina alla box
                head_cmd = JointTrajectory()
                head_cmd.joint_names = ["head_1_joint", "head_2_joint"]
                point = JointTrajectoryPoint()
                point.positions = [0.0, -0.5 * (object_size / max_size)]  # Regola la posizione della testa
                point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
                head_cmd.points = [point]
                self.head_pub.publish(head_cmd)

                self.get_logger().info(f"Blocco rilevato: x={object_center_x}, velocità: {cmd.linear.x:.2f}, rotazione: {cmd.angular.z:.2f}")

                # Ferma la rotazione
                self.rotating = False
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)

                # Salva la posizione della box
                self.box_detected = True
                self.box_position = (x, y, w, h, angle)
            else:
                # Se il contorno non è abbastanza grande, continua a ruotare
                if self.rotating:
                    cmd.angular.z = -0.3  # Ruota verso destra (da regolare)
                    self.cmd_vel_pub.publish(cmd)

        else:
            # Se non vede nulla, continua a ruotare
            if self.rotating:
                cmd.angular.z = -0.3  # Ruota verso destra (da regolare)
                self.cmd_vel_pub.publish(cmd)

        # Mostra l'immagine per debug
        cv2.imshow("Blue Block Detection", cv_image)
        cv2.waitKey(1)

    def scan_callback(self, msg):
        if not self.box_detected:
            return

        # Calcola la distanza minima rilevata dal laser
        min_distance = min(msg.ranges)

        # Se la distanza minima è maggiore della soglia, avvicinati alla box
        if min_distance > self.threshold_distance:
            cmd = Twist()
            cmd.linear.x = 0.2  # Velocità di avvicinamento (da regolare)
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
        else:
            # Ferma il robot se è abbastanza vicino
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Robot fermato a distanza di sicurezza dalla box")

            # Regola l'orientamento del robot per essere parallelo alla faccia della box
            angle = self.box_position[4]
            cmd.angular.z = np.deg2rad(angle)
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f"Robot orientato parallelo alla box con angolo: {angle} gradi")

def main(args=None):
    rclpy.init(args=args)
    node = BlueBlockFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()