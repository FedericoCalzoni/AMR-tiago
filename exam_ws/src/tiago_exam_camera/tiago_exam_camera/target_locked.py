import rclpy, cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster

class ArucoCubeDetection(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.camera_info_subscription = self.create_subscription(CameraInfo, '/head_front_camera/rgb/camera_info', self.camera_info_callback, 1)
        self.image_subscription = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 1)
        self.joint_state_subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.aruco_publisher = self.create_publisher(TransformStamped, '/aruco_single/transform', 10)
        self.head_state = JointTrajectory()
        self.head_state.joint_names = ['head_1_joint', 'head_2_joint']
        self.head_publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.bridge = CvBridge()
                
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()        
        
        self.goal_found = False
        self.camera_info = None
        self.current_position = [0.0, 0.0]  # Initialize head position
        self.target_ids = [63, 582]
        self.marker_size = 0.06
        
        self.get_logger().info('ArUco Detector Node initialized')

    def camera_info_callback(self, msg):
        self.camera_info = msg
        
    def image_callback(self, msg):
        
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        # red dot in the center of the image
        cv2.circle(self.img, (self.img.shape[1] // 2, self.img.shape[0] // 2), 2, (0, 0, 255), -1)

        if ids is not None:
            self.get_logger().info(f"Cubes locked, IDs: {ids}")
            
            for i, id in enumerate(ids):
                if id[0] == self.target_ids[0]:  # ID 63
                    corner = corners[i]
                    cv2.aruco.drawDetectedMarkers(self.img, corners, ids)
                    # Calculate the center of the marker
                    center_x = int(corner[0][0][0] + corner[0][2][0]) // 2
                    center_y = int(corner[0][0][1] + corner[0][2][1]) // 2
                    
                    self.goal_found = True
                    self.move_head(center_x, center_y)
                    
                    # cv2.circle(self.img, (center_x, center_y), 2, (255, 0, 0), -1)
                    
                    self.publish_marker_transform(corner, id[0], msg.header.stamp)
        elif not self.goal_found:
            self.rotate_in_place()
        else:
            self.get_logger().warn("Target lost")
        
        cv2.imshow("Tiago's view", self.img)
        cv2.waitKey(1)
        
    def joint_state_callback(self, msg):
        # Extract head joint positions from joint states message
        try:
            head_1_index = msg.name.index('head_1_joint')
            head_2_index = msg.name.index('head_2_joint')
            
            self.current_position[0] = msg.position[head_1_index]
            self.current_position[1] = msg.position[head_2_index]
            self.head_position_initialized = True
            
            # Log only the first time or occasionally for debugging
            if not hasattr(self, 'logged_head_position') or not self.logged_head_position:
                self.get_logger().info(f'Current head position: [{self.current_position[0]:.3f}, {self.current_position[1]:.3f}]')
                self.logged_head_position = True
        except ValueError:
            # If head joint names are not found in the message
            self.get_logger().warn('Head joint names not found in joint_states message')

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
        Kp = 0.001  # Proportional gain for head movement
        Ki = 0.001  # Integral gain for head movement
        Kd = 0.001  # Derivative gain for head movement
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
        
    def publish_marker_transform(self, corner,  marker_id, timestamp):
        
        if self.camera_info is None:
            self.get_logger().warn("Camera info not yet received. Skipping transform publishing.")
            return
        
        camera_matrix = np.array(self.camera_info.k).reshape(3, 3)
        dist_coeffs = np.array(self.camera_info.d)
        
        # Get the marker pose relative to the camera
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([corner[0]], 
                                                              self.marker_size, 
                                                              camera_matrix, 
                                                              dist_coeffs)
        
        # Create transform message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'head_front_camera_rgb_optical_frame'
        transform_msg.child_frame_id = f'aruco_marker_{marker_id}'
        
        # Set translation
        transform_msg.transform.translation.x = float(tvecs[0][0][0])
        transform_msg.transform.translation.y = float(tvecs[0][0][1])
        transform_msg.transform.translation.z = float(tvecs[0][0][2])
        
        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvecs[0][0])
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()  # x, y, z, w
        
        transform_msg.transform.rotation.x = float(quat[0])
        transform_msg.transform.rotation.y = float(quat[1])
        transform_msg.transform.rotation.z = float(quat[2])
        transform_msg.transform.rotation.w = float(quat[3])
        
        # Publish the transform
        self.aruco_publisher.publish(transform_msg)
        self.get_logger().info(f"Published transform for ID {marker_id}")
        
        
def main(args=None):
    rclpy.init(args=args)

    locking_target = ArucoCubeDetection()

    rclpy.spin(locking_target)
    locking_target.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()