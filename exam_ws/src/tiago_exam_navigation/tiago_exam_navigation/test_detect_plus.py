import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class BlueBlockFollower(Node):
    def __init__(self):
        super().__init__('blue_block_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.box_detected = False
        self.box_position = None
        self.threshold_distance = 0.5  # Distanza di sicurezza dalla box (da regolare)
        self.rotate_duration = 2.0  # Durata della rotazione in secondi (da regolare)
        self.rotate_start_time = self.get_clock().now()
        self.rotating = True

    def image_callback(self, msg):
        if self.box_detected or self.rotating:
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
                self.cmd_vel_pub.publish(cmd)

                # Salva la posizione della box
                self.box_detected = True
                self.box_position = (object_center_x, y, w, h)
                self.get_logger().info(f"Blocco rilevato: x={object_center_x}, larghezza: {w}, altezza: {h}")

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

    def timer_callback(self):
        if self.rotating:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.rotate_start_time).nanoseconds / 1e9
            if elapsed_time < self.rotate_duration:
                cmd = Twist()
                cmd.angular.z = -0.3  # Ruota verso destra (da regolare)
                self.cmd_vel_pub.publish(cmd)
            else:
                self.rotating = False
                cmd = Twist()
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                self.get_logger().info("Rotazione iniziale completata")

def main(args=None):
    rclpy.init(args=args)
    node = BlueBlockFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()