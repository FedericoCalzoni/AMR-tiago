import rclpy, cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped

class ArucoCubeDetection(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.callback, 1)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.aruco_publisher = self.create_publisher(TransformStamped, '/aruco_single/transform', 10)
        self.head_state = JointTrajectory()
        self.head_state.joint_names = ['head_1_joint', 'head_2_joint']
        self.current_position = [0.0, 0.0]  # Initialize head position
        self.head_publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        
        self.aruco_params = cv2.aruco.DetectorParameters_create()        
        self.aruco_params.minDistanceToBorder =  7
        self.aruco_params.cornerRefinementMaxIterations = 149
        self.aruco_params.minOtsuStdDev= 4.0
        self.aruco_params.adaptiveThreshWinSizeMin= 7
        self.aruco_params.adaptiveThreshWinSizeStep= 49
        self.aruco_params.minMarkerDistanceRate= 0.014971725679291437
        self.aruco_params.maxMarkerPerimeterRate= 10.075976700411534 
        self.aruco_params.minMarkerPerimeterRate= 0.2524866841549599 
        self.aruco_params.polygonalApproxAccuracyRate= 0.05562707541937206
        self.aruco_params.cornerRefinementWinSize= 9
        self.aruco_params.adaptiveThreshConstant= 9.0
        self.aruco_params.adaptiveThreshWinSizeMax= 369
        self.aruco_params.minCornerDistanceRate= 0.09167132584946237

        self.goal_found = False

    def callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        cv2.circle(self.img, (self.img.shape[1] // 2, self.img.shape[0] // 2), 2, (0, 0, 255), -1)

        target_ids = [63, 582]
        if ids is not None:
            self.goal_found = True
            self.get_logger().info(f"Cubes locked, IDs: {ids}")
            for id in ids:
                if id[0] == target_ids[0]:
                    for corner in corners:
                        cv2.aruco.drawDetectedMarkers(self.img, corners, ids)
                        # Calculate the center of the marker
                        center_x = int(corner[0][0][0] + corner[0][2][0]) // 2
                        center_y = int(corner[0][0][1] + corner[0][2][1]) // 2
                        # self.get_logger().info(f"Center: {center_x}, {center_y}")
                        cv2.circle(self.img, (center_x, center_y), 2, (255, 0, 0), -1)
                        #self.move_toward_marker(center_x)
                        self.move_head(center_x, center_y)
            else:
                self.get_logger().info(f"\033[93mNot 582 but found IDs: {ids}\033[0m")
        elif not self.goal_found:
            self.rotate_in_place()
        # else:
        #     self.goal_found = False
        
        
        cv2.imshow("Tiago's view", self.img)
        cv2.waitKey(1)

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
        
        # PID controller for head movement
        Kp = 0.0001  # Proportional gain for head movement
        Ki = 0.00001  # Integral gain for head movement
        Kd = 0.00001  # Derivative gain for head movement
        integral_limit = 100  # Limit for integral term to prevent wind-up

        # Initialize integral and derivative errors if they don't exist
        if not hasattr(self, 'integral_error_x'):
            self.integral_error_x = 0.0
        if not hasattr(self, 'integral_error_y'):
            self.integral_error_y = 0.0
        if not hasattr(self, 'previous_error_x'):
            self.previous_error_x = 0.0
        if not hasattr(self, 'previous_error_y'):
            self.previous_error_y = 0.0
        
        # Update integral errors with limit to prevent wind-up
        self.integral_error_x = max(min(self.integral_error_x + error_x, integral_limit), -integral_limit)
        self.integral_error_y = max(min(self.integral_error_y + error_y, integral_limit), -integral_limit)

        # Calculate derivative errors
        derivative_error_x = error_x - self.previous_error_x
        derivative_error_y = error_y - self.previous_error_y

        # Calculate control signals
        self.current_position[0] += -Kp * error_x - Ki * self.integral_error_x - Kd * derivative_error_x
        self.current_position[1] += -Kp * error_y - Ki * self.integral_error_y - Kd * derivative_error_y

        # Update previous errors
        self.previous_error_x = error_x
        self.previous_error_y = error_y

        point = JointTrajectoryPoint()
        # self.get_logger().info(f"Error: {error_x}, {error_y}")
        # self.get_logger().info(f"Move head: {self.current_position[0]}, {self.current_position[1]}")
        point.positions = self.current_position
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()

        self.head_state.points = [point]
        # self.get_logger().info(f"Head trajectory: {self.head_state}")
        self.head_publisher.publish(self.head_state)

def main(args=None):
    rclpy.init(args=args)

    locking_target = ArucoCubeDetection()

    rclpy.spin(locking_target)
    locking_target.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()