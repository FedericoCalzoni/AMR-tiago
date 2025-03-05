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
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.head_publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        
        self.aruco_params = cv2.aruco.DetectorParameters_create()        
        self.aruco_params.adaptiveThreshWinSizeMin = 3   # Smaller window to catch more potential markers
        self.aruco_params.adaptiveThreshWinSizeMax = 50  # Larger window to widen thresholding range
        self.aruco_params.adaptiveThreshWinSizeStep = 10 # Larger step for broader threshold exploration
        self.aruco_params.minMarkerPerimeterRate = 0.005 # Catch very small markers
        self.aruco_params.maxMarkerPerimeterRate = 6.0   # Detect very large markers
        self.aruco_params.polygonalApproxAccuracyRate = 0.05 # Loosen contour accuracy for broader matches
        self.aruco_params.minCornerDistanceRate = 0.05   # Allow corners to be closer
        self.aruco_params.minDistanceToBorder = 1        # Detect markers very close to the border
        self.aruco_params.minMarkerDistanceRate = 0.05   # Allow markers to be closer together


        self.goal_reached = False
        self.goal_found = False

    def callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # cv2.imshow("Tiago's view", self.img)
        # cv2.waitKey(1)
        
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            self.goal_found = True
            self.get_logger().info(f"Cubes locked, IDs: {ids}")
            for corner in corners:
                cv2.aruco.drawDetectedMarkers(self.img, corners, ids)
                # Calculate the center of the marker
                center_x = int(corner[0][0][0] + corner[0][2][0]) // 2
                center_y = int(corner[0][0][1] + corner[0][2][1]) // 2
                self.get_logger().info(f"Center: {center_x}, {center_y}")
                cv2.circle(self.img, (self.img.shape[1] // 2, self.img.shape[0] // 2), 5, (0, 0, 255), -1)
                cv2.circle(self.img, (center_x, center_y), 5, (250, 0, 0), -1)
                self.move_toward_marker(center_x)
                self.move_head(center_x, center_y)
            cv2.imshow("Tiago's view", self.img)
            cv2.waitKey(1)
        elif not self.goal_found:
            self.rotate_in_place()
        # else:
        #     self.goal_found = False
            
    def rotate_in_place(self):
        twist = Twist()
        twist.angular.z = -0.2
        self.twist_publisher.publish(twist)

    def move_toward_marker(self, center_x):
        twist = Twist()
        img_center_x = self.img.shape[1] // 2

        # Rotate to align the marker with the center of the image
        if center_x < img_center_x - 50:
            twist.angular.z = 0.2
        elif center_x > img_center_x + 50:
            twist.angular.z = -0.2
        else:
            twist.angular.z = 0.0

        # Move forward if the marker is centered
        if abs(center_x - img_center_x) < 50:
            twist.linear.x = 0.2
        else:
            twist.linear.x = 0.0

        self.twist_publisher.publish(twist)

    def move_head(self, center_x, center_y):
        img_center_x = self.img.shape[1] // 2
        img_center_y = self.img.shape[0] // 2
        
        # Calculate the error between the center of the image and the marker
        error_x = center_x - img_center_x
        error_y = center_y - img_center_y
        
        # TODO: It would be better to use a proportional controller here
        if error_x < -1:
            move_head_x = 0.2
        elif error_x > 1:
            move_head_x = -0.2
        else:
            move_head_x = 0.0
            
        if error_y < -1:
            move_head_y = 0.2
        elif error_y > 1:
            move_head_y = -0.2
        else:
            move_head_y = 0.0

        # Create a JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['head_1_joint', 'head_2_joint']

        point = JointTrajectoryPoint()
        self.get_logger().info(f"Error: {error_x}, {error_y}")
        self.get_logger().info(f"Move head: {move_head_x}, {move_head_y}")
        point.positions = [move_head_x,move_head_y]
        point.time_from_start = rclpy.duration.Duration(seconds=0.5).to_msg()

        trajectory_msg.points.append(point)
        self.get_logger().info(f"Head trajectory: {trajectory_msg}") # il messaggio lo manda ma non lo riceve qualche volta?
        self.head_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)

    locking_target = ArucoCubeDetection()

    rclpy.spin(locking_target)
    locking_target.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()