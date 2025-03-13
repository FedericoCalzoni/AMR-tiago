import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge
import time

class BlueBlockNavigator(Node):
    def __init__(self):
        super().__init__('blue_block_navigator')
        self.bridge = CvBridge()
        
        # Subscriber alla fotocamera
        self.image_sub = self.create_subscription(
            Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)

        # Publisher per i comandi di navigazione e velocità
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variabili di stato
        self.detected = False  # True quando rileva il blocco
        self.block_centered = False  # True quando il blocco è centrato
        self.moving_to_block = False  # True quando sta navigando
        self.image_width = 640  # Larghezza standard della fotocamera (da aggiornare se diversa)
        self.stop_distance = 0.5  # Distanza minima dal blocco prima di fermarsi (in metri)

    def image_callback(self, msg):
        # Converti l'immagine ROS in OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Converti in HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Intervallo di colore per il blu
        lower_blue = np.array([90, 50, 50])  
        upper_blue = np.array([130, 255, 255])  

        # Maschera per il colore blu
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Trova i contorni dell'oggetto blu
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Trova il contorno più grande
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Centro della scatola
            block_center_x = x + w // 2

            # Disegna il rettangolo sulla scatola rilevata
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Invia un messaggio nel log
            self.get_logger().info(f"Blocco rilevato a: x={block_center_x}, larghezza={self.image_width}")

            # 1. Se il blocco non è stato ancora rilevato, inizia a cercarlo
            if not self.detected:
                self.detected = True
                self.get_logger().info("Blocco rilevato, centraggio in corso...")

            # 2. Se il blocco è stato rilevato, continua a ruotare finché non è centrato
            if abs(block_center_x - self.image_width // 2) > 30:  # Tolleranza di 30 pixel
                self.align_block(block_center_x)
            else:
                self.block_centered = True
                self.get_logger().info("Blocco centrato, avvio navigazione...")
                self.navigate_to_block(x, y)

        # Mostra l'immagine per debug
        cv2.imshow("Blue Block Detection", cv_image)
        cv2.waitKey(1)

    def start_rotation(self):
        """Fa ruotare Tiago finché non rileva il blocco."""
        twist = Twist()
        twist.angular.z = 0.6  # Velocità di rotazione
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Tiago sta ruotando in cerca del blocco...")

    def align_block(self, block_center_x):
        """Regola la rotazione per centrare la scatola nell'immagine."""
        twist = Twist()
        error = block_center_x - self.image_width // 2  # Differenza tra centro blocco e centro immagine

        # Se l'errore è positivo, ruota a sinistra, altrimenti ruota a destra
        twist.angular.z = -0.2 if error > 0 else 0.2  
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Allineamento blocco: errore={error}")

    def navigate_to_block(self, x, y):
        """Invia il goal al planner per navigare verso il blocco."""
        if self.moving_to_block:
            return  # Evita di inviare più volte il goal

        self.moving_to_block = True

        # Converti le coordinate in metri
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x / 100  # Da pixel a metri
        goal.pose.position.y = y / 100
        goal.pose.orientation.w = 1.0

        # Pubblica l'obiettivo
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Inizio navigazione verso x={goal.pose.position.x}, y={goal.pose.position.y}")

        # Controlla quando fermarsi
        self.create_timer(0.5, self.check_distance_to_block)

    def check_distance_to_block(self):
        """Controlla se Tiago è vicino alla scatola e ferma la navigazione se necessario."""
        # Qui dovresti recuperare la distanza dal blocco usando odometria o LIDAR (non incluso)
        # Supponiamo di avere una funzione fittizia che ci dia la distanza
        distance = self.get_distance_to_block()  # Dovrebbe essere implementata con odometria o sensori

        if distance and distance < self.stop_distance:
            self.get_logger().info(f"Arrivato vicino alla scatola! Distanza: {distance}m. Fermando navigazione.")
            self.cmd_vel_pub.publish(Twist())  # Ferma il movimento
            self.moving_to_block = False

    def get_distance_to_block(self):
        """Placeholder per ottenere la distanza dalla scatola."""
        # Qui potresti implementare una funzione che usa il LIDAR o l'odometria
        # Per ora restituiamo un valore fittizio (da sostituire con una misura reale)
        return 0.4  # Supponiamo che siamo vicini al blocco


def main(args=None):
    rclpy.init(args=args)
    node = BlueBlockNavigator()

    # Inizia la rotazione iniziale
    node.start_rotation()

    rclpy.spin(node)

    # Ferma Tiago prima di chiudere
    node.cmd_vel_pub.publish(Twist())  
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
