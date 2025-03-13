import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge

class BlueBlockNavigator(Node):
    def __init__(self):
        super().__init__('blue_block_navigator')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/head_front_camera/depth_registered/image_raw', self.depth_callback, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variabili di stato
        self.detected = False  # Se il blocco è stato rilevato
        self.block_centered = False  # Se il blocco è centrato
        self.moving_to_block = False  # Se si sta navigando verso il blocco
        self.image_width = 640  # Larghezza immagine
        self.stop_distance = 0.5  # Distanza minima dalla scatola per fermarsi
        self.current_distance = None  # Ultima distanza rilevata dalla depth camera

        # Inizia la rotazione per cercare il blocco
        self.rotate_search()

        # Timer per controllare periodicamente la ricerca e la navigazione
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        """Chiamato periodicamente per eseguire operazioni come la ricerca del blocco."""
        if not self.detected:
            self.rotate_search()
        elif self.block_centered and self.current_distance and self.current_distance > self.stop_distance:
            self.navigate_to_block()

    def image_callback(self, msg):
        """Processa l'immagine RGB per trovare il blocco blu."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Definizione della soglia per il colore blu
        lower_blue = np.array([90, 50, 50])  
        upper_blue = np.array([130, 255, 255])  

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Trova il contorno più grande
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            block_center_x = x + w // 2
            block_center_y = y + h // 2  # Aggiungiamo la coordinata Y per la depth

            # Memorizza la posizione della scatola
            self.last_block_position = (block_center_x, block_center_y)

            # Disegna il rettangolo sulla scatola rilevata
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.line(cv_image, (block_center_x, 0), (block_center_x, self.image_width), (0, 0, 255), 2)

            self.get_logger().info(f"Blocco rilevato a x={block_center_x}")

            # Se il blocco non è stato ancora rilevato, fermare la rotazione
            if not self.detected:
                self.detected = True
                self.stop_robot()
                self.get_logger().info("🔵 Blocco rilevato! Inizio centraggio...")

            # Se il blocco è stato rilevato, ma non è centrato, allinearlo
            if abs(block_center_x - self.image_width // 2) > 30:
                self.align_block(block_center_x)
            else:
                self.block_centered = True
                self.get_logger().info("🎯 Blocco centrato! Verifica distanza...")

        # Mostra l'immagine per debug
        cv2.imshow("Blue Block Detection", cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        """Legge la distanza dal blocco dalla depth camera."""
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        h, w = depth_image.shape
        center_x, center_y = w // 2, h // 2

        distance = depth_image[center_y, center_x]

        if np.isfinite(distance):
            self.current_distance = distance
            self.get_logger().info(f"📏 Distanza dalla scatola: {distance:.2f} m")

            if self.block_centered and distance < self.stop_distance:
                self.get_logger().info("🛑 Fermata automatica vicino alla scatola!")
                self.stop_robot()

    def rotate_search(self):
        """Continua a ruotare finché non trova il blocco."""
        if not self.detected:  # Se non ha trovato il blocco, continua a ruotare
            twist = Twist()
            twist.angular.z = -0.5  # Velocità di rotazione
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Tiago sta ruotando in cerca del blocco...")
        else:
            self.stop_robot()  # Ferma la rotazione quando il blocco è trovato
            self.get_logger().info("Blocco trovato! Rotazione interrotta.")

    def stop_robot(self):
        """Ferma immediatamente il robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)  # Pubblica velocità zero
        self.get_logger().info("🛑 Robot fermato!")

    def align_block(self, block_center_x):
        """Regola la rotazione per centrare il blocco nell'immagine."""
        twist = Twist()
        error = block_center_x - self.image_width // 2  # Differenza tra centro blocco e centro immagine
        twist.angular.z = -0.2 if error > 0 else 0.2  
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"🔄 Allineamento in corso: errore={error}")

    def navigate_to_block(self):
        """Naviga verso il blocco usando il planner."""
        if self.moving_to_block:
            return  # Evita di inviare più volte il goal

        self.moving_to_block = True

        # Crea un goal per il navigatore
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self.current_distance - 0.3  # Si ferma leggermente prima
        goal.pose.orientation.w = 1.0

        # Pubblica il goal
        self.goal_pub.publish(goal)
        self.get_logger().info(f"🚀 Navigazione verso il blocco! Target: x={goal.pose.position.x}m")

        # Controlla la distanza periodicamente
        self.create_timer(0.5, self.check_distance_to_block)

    def check_distance_to_block(self):
        """Verifica la distanza attuale e ferma Tiago se necessario."""
        if self.current_distance and self.current_distance < self.stop_distance:
            self.get_logger().info(f"✅ Arrivato! Distanza attuale: {self.current_distance:.2f}m")
            self.stop_robot()
            self.moving_to_block = False


def main(args=None):
    rclpy.init(args=args)
    node = BlueBlockNavigator()
    rclpy.spin(node)
    node.stop_robot()  # Ferma il robot prima di chiudere
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
