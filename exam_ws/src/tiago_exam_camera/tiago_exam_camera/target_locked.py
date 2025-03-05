import rclpy, cv2, numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArucoCubeDetection(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/rgb/image_raw',
            self.callback,
            1)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.head_publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

        self.aruco_dicts = {
            "DICT_4X4_50": cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50),
            "DICT_4X4_100": cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100),
            "DICT_4X4_250": cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250),
            "DICT_4X4_1000": cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000),
            "DICT_5X5_50": cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50),
            "DICT_5X5_100": cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100),
            "DICT_5X5_250": cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250),
            "DICT_5X5_1000": cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000),
            "DICT_6X6_50": cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50),
            "DICT_6X6_100": cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100),
            "DICT_6X6_250": cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250),
            "DICT_6X6_1000": cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000),
            "DICT_7X7_50": cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50),
            "DICT_7X7_100": cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_100),
            "DICT_7X7_250": cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_250),
            "DICT_7X7_1000": cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_1000),
            "DICT_ARUCO_ORIGINAL": cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        }
        
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.adaptiveThreshWinSizeMin = 5  # Increase for better thresholding
        self.aruco_params.adaptiveThreshWinSizeMax = 25  # Increase for better thresholding
        self.aruco_params.adaptiveThreshWinSizeStep = 5  # Decrease for more precise thresholding
        self.aruco_params.minMarkerPerimeterRate = 0.02  # Decrease for smaller markers
        self.aruco_params.maxMarkerPerimeterRate = 4.5  # Increase for larger markers
        self.aruco_params.polygonalApproxAccuracyRate = 0.03  # Decrease for more accurate contour
        self.aruco_params.minCornerDistanceRate = 0.1  # Increase for larger corner distances
        self.aruco_params.minDistanceToBorder = 5  # Increase for markers closer to the border
        self.aruco_params.minMarkerDistanceRate = 0.1  # Increase for markers farther apart

        self.goal_reached = False

    def callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        cv2.imshow("Tiago's view", self.img)
        cv2.waitKey(1)
        
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
        if ids is not None:
            self.get_logger().info(f"Cubes locked, IDs: {ids}")
            #if 63 in ids:
            #    self.get_logger().info(f"EUREKA 63 LOCKED")
            if 586 in ids:
                self.get_logger().info(f"ZIOPERA 586 LOCKED")
            for corner in corners:
                cv2.aruco.drawDetectedMarkers(self.img, corners, ids)
                # Calculate the center of the marker
                center_x = int(corner[0][0][0] + corner[0][2][0]) // 2
                center_y = int(corner[0][0][1] + corner[0][2][1]) // 2
                cv2.circle(self.img, (center_x, center_y), 5, (0, 255, 0), -1)
                #self.move_toward_marker(center_x, center_y)
                #self.move_head(center_x, center_y)

    def move_toward_marker(self, center_x):
        twist = Twist()
        img_center_x = self.img.shape[1] // 2

        # Rotate to align the marker with the center of the image
        if center_x < img_center_x - 50:
            twist.angular.z = 0.1
        elif center_x > img_center_x + 50:
            twist.angular.z = -0.1
        else:
            twist.angular.z = 0.0

        # Move forward if the marker is centered
        if abs(center_x - img_center_x) < 50:
            twist.linear.x = 0.1
        else:
            twist.linear.x = 0.0

        self.publisher.publish(twist)

    def move_head(self, center_x, center_y):
        img_center_x = self.img.shape[1] // 2
        img_center_y = self.img.shape[0] // 2

        # Calculate the error between the center of the image and the marker
        error_x = center_x - img_center_x
        error_y = center_y - img_center_y

        # Create a JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['head_1_joint', 'head_2_joint']

        point = JointTrajectoryPoint()
        point.positions = [error_x * 0.001, error_y * 0.001]  # Scale the error to joint positions
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()

        trajectory_msg.points.append(point)
        self.head_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)

    locking_target = ArucoCubeDetection()

    rclpy.spin(locking_target)
    locking_target.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()