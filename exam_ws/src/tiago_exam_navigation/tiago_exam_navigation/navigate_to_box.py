import rclpy, cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class NavigateToBox(Node):
    def __init__(self):
        super().__init__('navigate_to_box')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.twist = Twist()
        self.box_detected = False

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        edged_image = cv2.Canny(blurred_image, 50, 150)
        contours, _ = cv2.findContours(edged_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            self.box_detected = True
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            self.align_to_box(x, w, cv_image.shape[1])
        else:
            self.box_detected = False
            self.spin()

        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

    def spin(self):
        self.twist.angular.z = -0.2
        self.publisher_.publish(self.twist)

    def align_to_box(self, x, w, image_width):
        box_center = x + w / 2
        image_center = image_width / 2
        error = box_center - image_center

        if abs(error) > 10:
            self.twist.angular.z = -0.002 * error
        else:
            self.twist.angular.z = 0.0
            self.move_forward()

        self.publisher_.publish(self.twist)

    def move_forward(self):
        self.twist.linear.x = 0.2
        self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToBox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()