import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class BlueBlockDetector(Node):
    def __init__(self):
        super().__init__('blue_block_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        
        # Pubblicatore per inviare l'obiettivo al nodo di navigazione
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

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

            # Converti le coordinate del blocco in un obiettivo da inviare
            self.send_goal(x, y)

        # Mostra l'immagine per debug
        cv2.imshow("Blue Block Detection", cv_image)
        cv2.waitKey(1)

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"  # Coordinate nel sistema di riferimento "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        # Le coordinate rilevate sono in pixel, devi convertirle in metri (ad esempio, se ogni pixel è 0.01 m)
        goal.pose.position.x = x / 100  # Conversione da pixel a metri
        goal.pose.position.y = y / 100  # Conversione da pixel a metri
        goal.pose.orientation.w = 1.0  # Orientamento neutro (puoi modificarlo in base alla necessità)

        # Pubblica l'obiettivo al topic di navigazione
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Obiettivo inviato a: x={goal.pose.position.x}, y={goal.pose.position.y}")


def main(args=None):
    rclpy.init(args=args)
    node = BlueBlockDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
