import rclpy, cv2, math, time
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import JointState
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
        self.marker_size = 0.04667
        self.get_logger().info('ArUco Detector Node initialized')
        # Camera parameters
        self.camera_matrix = np.array([
            [522.1910329546544, 0.0, 320.5],
            [0.0, 522.1910329546544, 240.5],
            [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([1.0e-08, 1.0e-08, 1.0e-08, 1.0e-08, 1.0e-08])

    def camera_info_callback(self, msg):
        self.camera_info = msg
        
    def image_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        # red dot in the center of the image
        cv2.circle(self.img, (self.img.shape[1] // 2, self.img.shape[0] // 2), 2, (0, 0, 255), -1)
        
        # Initialize tracking memory if not present
        if not hasattr(self, 'last_best_id_idx'):
            self.last_best_id_idx = None
            self.lost_track_count = 0
            self.track_memory = 15  # Number of frames to remember previous detection
        
        if ids is not None:
            # Filter only markers with our target ID
            target_indices = [i for i, id in enumerate(ids) if id[0] == self.target_ids[1]]
            if target_indices:
                lowest_x_normal = float('inf')
                current_best_id_idx = None

                for i in target_indices:
                    marker_size = self.marker_size
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], marker_size, self.camera_matrix, self.dist_coeffs
                    )
                    rot_matrix, _ = cv2.Rodrigues(rvec[0])
                    normal = rot_matrix[:, 2]
                    x_component = abs(normal[0])
                    max_x_component_threshold = 0.3
                    if x_component < max_x_component_threshold:
                        if self.last_best_id_idx is not None and i == self.last_best_id_idx:
                            # Only switch if new marker has significantly lower x component
                            if x_component < lowest_x_normal * 0.95:
                                lowest_x_normal = x_component
                                current_best_id_idx = i
                        elif x_component < lowest_x_normal:
                            lowest_x_normal = x_component
                            current_best_id_idx = i
                    if current_best_id_idx is not None:
                        best_id_idx = current_best_id_idx
                        self.last_best_id_idx = best_id_idx  # Update tracking memory
                        self.lost_track_count = 0  # Reset lost track counter
                        cv2.aruco.drawAxis(self.img, self.camera_matrix, self.dist_coeffs, 
                                        rvec, tvec, self.marker_size/2)
                        corner = corners[best_id_idx][0]
                        center_x = int((corner[0][0] + corner[2][0]) / 2)
                        center_y = int((corner[0][1] + corner[2][1]) / 2)
                        
                        self.goal_found = True
                        self.move_head(center_x, center_y)
                        #self.publish_marker_transform(corners[best_id_idx], ids[best_id_idx][0], msg.header.stamp)
                
            elif self.last_best_id_idx is not None and self.lost_track_count < self.track_memory:
                self.lost_track_count += 1
                self.get_logger().warn(f"Temporarily lost track - holding position ({self.lost_track_count}/{self.track_memory})")
            elif not self.goal_found:
                self.rotate_head()
                self.last_best_id_idx = None
            else:
                self.last_best_id_idx = None 
        else:
            if self.last_best_id_idx is not None and self.lost_track_count < self.track_memory:
                self.lost_track_count += 1
                self.get_logger().warn(f"Temporarily lost track - holding position ({self.lost_track_count}/{self.track_memory})")
            else:
                self.last_best_id_idx = None 
                if not self.goal_found:
                    self.rotate_head()
        
        cv2.imshow("Tiago's view", self.img)
        cv2.waitKey(1)
        
    def joint_state_callback(self, msg):
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

    def rotate_head(self):
        # Calculate control signals
        self.current_position[1] += -0.3
        point = JointTrajectoryPoint()
        point.positions = self.current_position
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        self.head_state.points = [point]
        self.head_publisher.publish(self.head_state)
        if self.current_position[1] < -1:
            self.current_position[0] += -0.3
            point = JointTrajectoryPoint()
            point.positions = self.current_position
            point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
            self.head_state.points = [point]
            self.head_publisher.publish(self.head_state)

    def move_head(self, center_x, center_y):
        img_center_x = self.img.shape[1] // 2
        img_center_y = self.img.shape[0] // 2
        
        # Calculate the error between the center of the image and the marker
        error_x = center_x - img_center_x
        error_y = center_y - img_center_y
        
        # Define threshold - stop moving if error is small enough
        error_threshold = 15  # pixels
        
        # Check if error is within threshold
        if abs(error_x) < error_threshold and abs(error_y) < error_threshold:
            return  # Skip movement if close enough
        
        # Better tuned PID parameters
        Kp = 0.003  # Increased for faster response
        Ki = 0.0005  # Reduced to minimize overshoot
        Kd = 0.002  # Increased for better damping
        
        integral_limit = 50  # Reduced to prevent excessive integral buildup
        
        # Initialize integral and derivative errors if they don't exist
        if not hasattr(self, 'integral_error_x'):
            self.integral_error_x = 0.0
        if not hasattr(self, 'integral_error_y'):
            self.integral_error_y = 0.0
        if not hasattr(self, 'previous_error_x'):
            self.previous_error_x = 0.0
        if not hasattr(self, 'previous_error_y'):
            self.previous_error_y = 0.0
        if not hasattr(self, 'last_time'):
            self.last_time = time.time()
        
        # Calculate time delta for better derivative calculation
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Only update integral when error is small (anti-windup technique)
        if abs(error_x) < 100:  # Only integrate when error is reasonable
            self.integral_error_x = max(min(self.integral_error_x + error_x * dt, integral_limit), -integral_limit)
        else:
            self.integral_error_x = 0  # Reset integral when error is large
        if abs(error_y) < 100:
            self.integral_error_y = max(min(self.integral_error_y + error_y * dt, integral_limit), -integral_limit)
        else:
            self.integral_error_y = 0
        # Calculate derivative errors with time scaling
        if dt > 0:
            derivative_error_x = (error_x - self.previous_error_x) / dt
            derivative_error_y = (error_y - self.previous_error_y) / dt
        else:
            derivative_error_x = 0
            derivative_error_y = 0
        # Apply low-pass filter to derivative term to reduce noise
        derivative_filter = 0.7  # Filter coefficient
        if hasattr(self, 'filtered_derivative_x'):
            self.filtered_derivative_x = derivative_filter * self.filtered_derivative_x + (1 - derivative_filter) * derivative_error_x
            self.filtered_derivative_y = derivative_filter * self.filtered_derivative_y + (1 - derivative_filter) * derivative_error_y
        else:
            self.filtered_derivative_x = derivative_error_x
            self.filtered_derivative_y = derivative_error_y
        
        dx = -Kp * error_x - Ki * self.integral_error_x 
        dy = -Kp * error_y - Ki * self.integral_error_y 
        # Limit maximum movement per iteration
        max_movement = 0.05  # radians
        dx = max(min(dx, max_movement), -max_movement)
        dy = max(min(dy, max_movement), -max_movement)
        # Update position
        self.current_position[0] += dx
        self.current_position[1] += dy
        
        # Update previous errors
        self.previous_error_x = error_x
        self.previous_error_y = error_y
        
        # Create and publish trajectory
        point = JointTrajectoryPoint()
        point.positions = self.current_position
        point.time_from_start = rclpy.duration.Duration(seconds=0.2).to_msg()  # Reduced time for faster response
        
        self.head_state.points = [point]
        self.head_publisher.publish(self.head_state)
        
    # def publish_marker_transform(self, corner,  marker_id, timestamp):
    #     if self.camera_info is None:
    #         self.get_logger().warn("Camera info not yet received. Skipping transform publishing.")
    #         return
        
    #     camera_matrix = np.array(self.camera_info.k).reshape(3, 3)
    #     dist_coeffs = np.array(self.camera_info.d)
        
    #     # Get the marker pose relative to the camera
    #     rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers([corner[0]], 
    #                                                           self.marker_size, 
    #                                                           camera_matrix, 
    #                                                           dist_coeffs)
        
    #     # Create transform message
    #     transform_msg = TransformStamped()
    #     transform_msg.header.stamp = self.get_clock().now().to_msg()
    #     transform_msg.header.frame_id = 'head_front_camera_rgb_optical_frame'
    #     transform_msg.child_frame_id = f'aruco_marker_{marker_id}'
        
    #     # Set translation
    #     transform_msg.transform.translation.x = float(tvecs[0][0][0])
    #     transform_msg.transform.translation.y = float(tvecs[0][0][1])
    #     transform_msg.transform.translation.z = float(tvecs[0][0][2])
        
    #     # Convert rotation vector to quaternion
    #     rotation_matrix, _ = cv2.Rodrigues(rvecs[0][0])
    #     r = R.from_matrix(rotation_matrix)
    #     quat = r.as_quat()  # x, y, z, w
        
    #     transform_msg.transform.rotation.x = float(quat[0])
    #     transform_msg.transform.rotation.y = float(quat[1])
    #     transform_msg.transform.rotation.z = float(quat[2])
    #     transform_msg.transform.rotation.w = float(quat[3])
        
    #     # Publish the transform
    #     self.aruco_publisher.publish(transform_msg)
    #     self.get_logger().info(f"Published transform for ID {marker_id}")
    
    def publish_marker_transform(self, corner,  marker_id, timestamp):
        if self.camera_info is None:
            self.get_logger().warn("Camera info not yet received. Skipping transform publishing.")
            return
        
        camera_matrix = np.array(self.camera_info.k).reshape(3, 3)
        dist_coeffs = np.array(self.camera_info.d)

        # Get the marker pose relative to the camera
        self.get_logger().info(f"Corner: {corner}")
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 
                                                              self.marker_size, 
                                                              camera_matrix, 
                                                              dist_coeffs)
        
        # Create transform message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = 'head_front_camera_optical_frame'
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