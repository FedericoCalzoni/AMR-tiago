import rclpy, cv2, time
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs_py.point_cloud2 import read_points
import open3d as o3d

from tiago_interfaces.msg import BoxInfo, FaceInfo  # Import the custom message types
from geometry_msgs.msg import Point

class NavigateToBox(Node):
    def __init__(self):
        super().__init__('navigate_to_box')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.faces_publisher_ = self.create_publisher(BoxInfo, '/box/faces_info', 10)
        self.z_face_publisher_ = self.create_publisher(FaceInfo, '/box/z_face_info', 10)
        self.create_subscription(Odometry, '/ground_truth_odom', self.odom_callback, 10)
        self.done = self.create_publisher(Bool, '/nav_to_box/done', 10)
        self.subscription = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        self.depth_subscription = self.create_subscription(PointCloud2, '/head_front_camera/depth_registered/points', self.depth_callback, 10)
        self.bridge = CvBridge()
        self.twist = Twist()
        self.faces_info_sent = False
        self.box_detected = False
        self.depth_data = None
        self.not_read = False
        self.moving = False

        # Ensure Tiago's head is at position (0, 0)
        self.head_state = JointTrajectory()
        self.head_state.joint_names = ['head_1_joint', 'head_2_joint']
        self.head_publisher = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.reset_head_position()

    def reset_head_position(self):
        point = JointTrajectoryPoint()
        self.current_position = [0.0, 0.0]  # Initialize current position
        point.positions = self.current_position
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        self.head_state.points = [point]
        self.head_publisher.publish(self.head_state)

    def move_head(self, center_x, center_y):
        img_center_x = self.img.shape[1] // 2
        img_center_y = self.img.shape[0] // 2
        cv2.circle(self.img, (img_center_x, img_center_y), 3, (0, 255, 255), -1)
        
        # Calculate the error between the center of the image and the marker
        error_x = center_x - img_center_x
        error_y = center_y - img_center_y
        
        # Define threshold - stop moving if error is small enough
        error_threshold = 50  # pixels
        
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

    def odom_callback(self, msg):
        if msg.twist.twist.linear.x != 0 and msg.twist.twist.linear.y != 0:
            self.moving = True
        else:
            self.moving = False

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img = cv_image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_cyan = np.array([80, 100, 100])
        upper_cyan = np.array([100, 255, 255])
        mask = cv2.inRange(hsv_image, lower_cyan, upper_cyan)
        masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        gray_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        edged_image = cv2.Canny(blurred_image, 50, 150)
        contours, _ = cv2.findContours(edged_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            self.box_detected = True
            min_area = 100
            large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
            if large_contours:
                largest_contour = max(large_contours, key=cv2.contourArea)
                cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 2)
                if self.depth_data is not None:
                    mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
                    cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)
                    xs,ys = np.where(mask == 255)
                    coordinates = list(zip(xs,ys))
                    
                    # Acquire point cloud
                    points = list(read_points(self.depth_data, field_names=("x", "y", "z"), skip_nans=True))
                    submatrix = np.array([point.item() for point in points]).reshape((480, 640, 3))
                    submatrix_masked = []
                    for c in coordinates:
                        submatrix_masked.append(submatrix[c])
                    submatrix_masked = np.array(submatrix_masked)

                    # Detect the faces of the box
                    faces = self.detect_box_faces(submatrix_masked)

                    # Log information about the detected faces
                    #print(f"Detected {len(faces)} faces:")
                    #for i, face in enumerate(faces):
                    #    self.get_logger().info(f"Face {i+1}:")
                    #    self.get_logger().info(f"  Normal: {face['normal']}")
                    #    self.get_logger().info(f"  Center: {face['center']}")
                    #    self.get_logger().info(f"  Points: {len(face['points'])}")
                    i = 0
                    for face in faces:
                        if i == 0:
                            # Color code: BGR
                            color = (0, 0, 255)
                        elif i == 1:
                            color = (0, 255, 0)
                        else:
                            color = (255, 0, 0)
                        i += 1
                        for x in face['points']:
                            px_x, px_y = self.from_mt_to_pixel(x)
                            cv_image[px_y, px_x] = color

                    # Publish faces information - custom message
                    b_c_x, b_c_y = self.publish_faces_info(faces, cv_image)
                    #self.move_head(b_c_x, b_c_y)
                    self.done.publish(Bool(data=True))
            elif not self.faces_info_sent:
                self.box_detected = True
                self.spin()
        elif not self.faces_info_sent:
            self.box_detected = True
            self.spin()

        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        #if self.not_read:
        self.depth_data = msg

    def from_mt_to_pixel(self, point):
        # Camera intrinsic parameters
        fx = 522.1910329546544  # Focal length in x
        fy = 522.1910329546544  # Focal length in y
        cx = 320.5  # Principal point x
        cy = 240.5  # Principal point y
        
        # Convert from meters to pixels
        px = int((point[0] * fx) / point[2] + cx)
        py = int((point[1] * fy) / point[2] + cy)
        
        return px, py

    def spin(self):
        self.twist.angular.z = -0.3
        self.publisher_.publish(self.twist)

    def publish_faces_info(self, faces, cv_image):
        box_faces_msg = BoxInfo()
        z_face_info_msg = FaceInfo()

        # Find the face whose normal is closest to the z-axis which in camera ref frame is y-axis
        y_axis = np.array([0, 1, 0])  # Standard y-axis
        z_face = max(faces, key=lambda face: np.dot(face['normal'], -y_axis))

        b_c_x, b_c_y = self.from_mt_to_pixel(z_face['center'])
        cv2.circle(cv_image, (b_c_x, b_c_y), 3, (0, 0, 0), -1)
        
        z_face_info_msg.face_number = "Z-axis face"
        z_face_info_msg.center = list(z_face['center'])
        z_face_info_msg.normal = list(z_face['normal'])
        z_face_info_msg.plane_coefficients = list(z_face['plane_coefficients'])
        z_face_info_msg.points = [Point(x=p[0], y=p[1], z=p[2]) for p in z_face['points']]
        self.z_face_publisher_.publish(z_face_info_msg)
        
        i = 0
        for face in faces:
            if all(face['center'][i] == z_face['center'][i] for i in range(3)):
                z_face_index = i
                break
            i += 1
        
        faces.pop(z_face_index)  # Remove the Z-axis face from the list
        for face in faces:
            i = 0
            if i == 0:
                color = (0, 0, 255)
            else:
                color = (0, 255, 0)
            
            c_x, c_y = self.from_mt_to_pixel(face['center'])
            cv2.circle(cv_image, (c_x,c_y), 3, (255, 255, 255), -1)
            
            face_info = FaceInfo()
            face_info.face_number = f"Face {i}, color [{color[0]}, {color[1]}, {color[2]}]"
            face_info.center = list(face['center'])
            face_info.normal = list(face['normal'])
            face_info.plane_coefficients = list(face['plane_coefficients'])
            face_info.points = [Point(x=p[0], y=p[1], z=p[2]) for p in face['points']]
            box_faces_msg.faces.append(face_info)
            i += 1
        self.faces_publisher_.publish(box_faces_msg)
        self.faces_info_sent = True
        return b_c_x, b_c_y

    def detect_box_faces(self, points, max_iterations=100, min_plane_points=10, distance_threshold=0.01):
        """
        Detect exactly three faces of a box from a matrix of 3D points.
        
        Parameters:
        -----------
        points : numpy.ndarray
            Matrix of 3D points (N x 3) where each row is a point (x, y, z)
        max_iterations : int
            Maximum number of iterations for plane detection
        min_plane_points : int
            Minimum number of points required to form a plane
        distance_threshold : float
            Maximum distance from a point to a plane to be considered an inlier
        
        Returns:
        --------
        faces : list of dict
            List of dictionaries, each containing:
            - 'plane_coefficients': [a, b, c, d] for the plane equation ax + by + cz + d = 0
            - 'points': numpy array of points belonging to the face
            - 'center': center point of the face
            - 'normal': normal vector of the face
        """
        # Convert to Open3D point cloud for easier processing
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # List to store detected faces
        faces = []
        remaining_points = np.array(points)
        
        # We'll use a sequential RANSAC approach to find exactly 3 planes
        for _ in range(3):  # Force detection of exactly 3 planes
            if len(remaining_points) < min_plane_points:
                break
                
            # Try to find a plane in the remaining points
            plane_model, inliers = self.fit_plane_sequential_ransac(remaining_points, distance_threshold, max_iterations)
            
            if len(inliers) >= min_plane_points:
                # Get the points that belong to this plane
                plane_points = remaining_points[inliers]
                
                # Calculate the center of the face
                center = np.mean(plane_points, axis=0)
                
                # Get the normal vector (first 3 coefficients of the plane equation)
                normal = plane_model[:3]
                
                # Ensure the normal points outward (assuming the box center is at the origin)
                if np.dot(normal, center) > 0:
                    normal = -normal
                    plane_model = -plane_model
                
                # Store the face information
                faces.append({
                    'plane_coefficients': plane_model,
                    'points': plane_points,
                    'center': center,
                    'normal': normal
                })
                
                # Remove the inliers from the remaining points
                remaining_points = remaining_points[~inliers]
        
        return faces

    def fit_plane_sequential_ransac(self, points, distance_threshold=0.01, max_iterations=100):
        """
        Fit a plane to 3D points using a sequential RANSAC approach.
        
        Parameters:
        -----------
        points : numpy.ndarray
            Matrix of 3D points (N x 3)
        distance_threshold : float
            Maximum distance from a point to a plane to be considered an inlier
        max_iterations : int
            Maximum number of RANSAC iterations
        
        Returns:
        --------
        plane_model : numpy.ndarray
            Plane coefficients [a, b, c, d] for the plane equation ax + by + cz + d = 0
        inliers : numpy.ndarray
            Boolean array indicating which points are inliers
        """
        best_plane = None
        best_inliers = None
        best_num_inliers = 0
        
        for _ in range(max_iterations):
            # Randomly select 3 points
            if len(points) < 3:
                break
                
            idx = np.random.choice(len(points), 3, replace=False)
            p1, p2, p3 = points[idx]
            
            # Compute the plane equation from these three points
            v1 = p2 - p1
            v2 = p3 - p1
            
            # Cross product to get the normal vector
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            
            # Check if the points are not collinear
            if norm < 1e-10:
                continue
                
            # Normalize the normal vector
            normal = normal / norm
            
            # Calculate the d component of the plane equation
            d = -np.dot(normal, p1)
            
            # Combine to get the plane equation coefficients
            plane = np.append(normal, d)
            
            # Calculate distances from all points to the plane
            distances = np.abs(np.dot(points, normal) + d)
            
            # Find inliers
            inliers = distances < distance_threshold
            num_inliers = np.sum(inliers)
            
            # Update best model if we found a better one
            if num_inliers > best_num_inliers:
                best_num_inliers = num_inliers
                best_plane = plane
                best_inliers = inliers
        
        # If we found a good plane, refine it using all inliers
        if best_plane is not None and best_num_inliers >= 3:
            # Extract the inlier points
            inlier_points = points[best_inliers]
            
            # Refine the plane using least squares
            refined_plane = self.refine_plane(inlier_points)
            
            # Recalculate inliers with the refined plane
            normal = refined_plane[:3]
            d = refined_plane[3]
            distances = np.abs(np.dot(points, normal) + d)
            best_inliers = distances < distance_threshold
            
            return refined_plane, best_inliers
        
        # If we didn't find a good plane, return None
        return None, np.zeros(len(points), dtype=bool)

    def refine_plane(self, points):
        """
        Refine a plane equation using least squares fitting.
        
        Parameters:
        -----------
        points : numpy.ndarray
            Matrix of 3D points (N x 3) that belong to the plane
        
        Returns:
        --------
        plane : numpy.ndarray
            Refined plane coefficients [a, b, c, d]
        """
        if len(points) < 3:
            return None
        
        # Calculate the centroid of the points
        centroid = np.mean(points, axis=0)
        
        # Center the points by subtracting the centroid
        centered_points = points - centroid
        
        # Perform SVD to find the normal vector
        u, s, vh = np.linalg.svd(centered_points)
        
        # The normal vector is the last column of vh
        normal = vh[2, :]
        
        # Ensure the normal is a unit vector
        normal = normal / np.linalg.norm(normal)
        
        # Calculate the d component
        d = -np.dot(normal, centroid)
        
        # Return the plane equation coefficients
        return np.append(normal, d)


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToBox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()