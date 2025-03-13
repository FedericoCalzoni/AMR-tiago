import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge
import time

class BlueBlockDetector(Node):
    def __init__(self):
        super().__init__('blue_block_detector')
        self.bridge = CvBridge()
        
        # Subscrizione alla fotocamera
        self.image_sub = self.create_subscription(
            Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)

        # Pubblicatori per l'obiettivo e la velocità
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variabile per determinare quando iniziare la pubblicazione dell'obiettivo
        self.detected = False
        self.start_time = None

        # Tempo massimo per la rotazione
        self.rotation_time_limit = 5.0  # in secondi

    def image_callback(self, msg):
        # Converti l'immagine ROS in un'immagine OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Converti l'immagine in spazio colore HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Definisci l'intervallo di colore per il celeste
        lower_blue = np.array([90, 50, 50])   # Min HSV per il blu chiaro
        upper_blue = np.array([130, 255, 255]) # Max HSV per il blu chiaro

        # Crea una maschera per il colore blu
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Trova i contorni dell'oggetto blu
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Trova il contorno più grande
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Disegna un rettangolo attorno all'oggetto rilevato
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.get_logger().info(f"Blocco celeste rilevato a: x={x}, y={y}")

            # Se il blocco è stato rilevato, imposta "detected" su True e salva il tempo
            if not self.detected:
                self.detected = True
                self.start_time = time.time()

            # Se il blocco è stato rilevato, pubblica l'obiettivo
            if self.detected and (time.time() - self.start_time) > self.rotation_time_limit:
                self.send_goal(x, y)

        # Mostra l'immagine per debug
        cv2.imshow("Blue Block Detection", cv_image)
        cv2.waitKey(1)

    def send_goal(self, x, y):
        # Converti le coordinate in metri (da pixel)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x / 100  # Conversione da pixel a metri
        goal.pose.position.y = y / 100  # Conversione da pixel a metri
        goal.pose.orientation.w = 1.0

        # Pubblica l'obiettivo al topic di navigazione
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Obiettivo inviato a: x={goal.pose.position.x}, y={goal.pose.position.y}")

    def start_rotation(self):
        # Comando di rotazione per far ruotare Tiago su se stesso
        twist = Twist()
        twist.angular.z = 0.2  # Velocità di rotazione (in radianti al secondo)

        # Pubblica il comando di velocità
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Tiago sta ruotando...")

def main(args=None):
    rclpy.init(args=args)
    node = BlueBlockDetector()

    # Inizia la rotazione all'avvio
    node.start_rotation()

    rclpy.spin(node)

    # Ferma Tiago e chiudi il nodo
    node.cmd_vel_pub.publish(Twist())  # Ferma il movimento
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# includere nel setup detect_move e questo 