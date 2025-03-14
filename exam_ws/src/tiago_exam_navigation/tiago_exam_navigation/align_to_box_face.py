import numpy as np
import math, cv2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tiago_interfaces.msg import BoxInfo, FaceInfo 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BoxFaceNavigator(Node):
    def __init__(self):
        super().__init__('box_face_navigator')
        
        # Create the action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF2 buffer and listener for coordinate transformations
        #self.tf_buffer = tf2_ros.Buffer()
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.face_subscriber_ = self.create_subscription(BoxInfo, '/box/faces_info', self.face_callback, 10)
        self.image_subscription = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.frame = None

    def image_callback(self, msg):
        if self.frame is not None:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            origin = self.from_mt_to_pixel(self.frame[3])
            cv2.circle(cv_image, (origin[0],origin[1]), 3, (255, 255, 255), -1)
            scale = 20  # Scale factor for the length of the arrows
            x_axis_end = (int(origin[0] + self.frame[0][0] * scale), int(origin[1] - self.frame[0][1] * scale))
            cv2.arrowedLine(cv_image, origin, x_axis_end, (0, 0, 255), 2)
            y_axis_end = (int(origin[0] + self.frame[1][0] * scale), int(origin[1] - self.frame[1][1] * scale))
            cv2.arrowedLine(cv_image, origin, y_axis_end, (0, 255, 0), 2)
            z_axis_end = (int(origin[0] + self.frame[2][0] * scale), int(origin[1] - self.frame[2][1] * scale))
            cv2.arrowedLine(cv_image, origin, z_axis_end, (255, 0, 0), 2)

            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)

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

    def face_callback(self, msg):
        # Extract the third face
        third_face = msg.faces[2]
        # Convert FaceInfo to a dictionary
        face_info = {
            'center': np.array([third_face.center[0], third_face.center[1], third_face.center[2]]),
            'normal': np.array([third_face.normal[0], third_face.normal[1], third_face.normal[2]]),
            'plane_coefficients': np.array([third_face.plane_coefficients[0], third_face.plane_coefficients[1], third_face.plane_coefficients[2]])
            #'points': np.array([[point[0], point[1], point[2]] for point in third_face.points])
        }

        # Create face reference frame
        self.create_frame(face_info)

        # Navigate to the third face
        #self.navigate_to_face(face_info)

    def create_frame(self, face_info): 
        face_center = face_info['center']
        face_normal = face_info['normal']
        
        # Define the z-axis as pointing upward
        z_axis = np.array([0, 0, 1])
        
        # Ensure the normal is a unit vector
        face_normal = face_normal / np.linalg.norm(face_normal)
        
        # Calculate the y-axis as the cross product of z-axis and face normal
        y_axis = np.cross(z_axis, face_normal)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # Recalculate the x-axis to ensure orthogonality
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        self.frame = [x_axis, y_axis, z_axis, face_center]
        
        # Create the transformation matrix
        #transformation_matrix = np.eye(4)
        #transformation_matrix[0:3, 0] = x_axis
        #transformation_matrix[0:3, 1] = y_axis
        #transformation_matrix[0:3, 2] = z_axis
        #transformation_matrix[0:3, 3] = face_center
        
        # Convert the transformation matrix to a PoseStamped message
        #face_frame_pose = PoseStamped()
        #face_frame_pose.header.frame_id = 'world'
        #face_frame_pose.header.stamp = self.get_clock().now().to_msg()
        
        #face_frame_pose.pose.position.x = face_center[0]
        #face_frame_pose.pose.position.y = face_center[1]
        #face_frame_pose.pose.position.z = face_center[2]
        
        # Convert rotation matrix to quaternion
        #quaternion = tf_transformations.quaternion_from_matrix(transformation_matrix)
        #face_frame_pose.pose.orientation.x = quaternion[0]
        #face_frame_pose.pose.orientation.y = quaternion[1]
        #face_frame_pose.pose.orientation.z = quaternion[2]
        #face_frame_pose.pose.orientation.w = quaternion[3]
        
        # Publish the frame for visualization
        #self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        #transform = tf2_geometry_msgs.PoseStamped_to_TransformStamped(face_frame_pose)
        #transform.child_frame_id = 'face_frame'
        #self.tf_broadcaster.sendTransform(transform)

    def calculate_approach_pose(self, face_info, camera_frame, robot_frame, approach_distance=0.5):
        """
        Calculate a suitable pose to approach the selected face.
        
        Parameters:
        -----------
        face_info : dict
            Dictionary containing face information with keys:
            - 'center': center point of the face
            - 'normal': normal vector of the face
        camera_frame : str
            Name of the camera frame (e.g., 'xtion_rgb_optical_frame')
        robot_frame : str
            Name of the robot's base frame (e.g., 'base_footprint')
        approach_distance : float
            Distance to maintain from the face (in meters)
            
        Returns:
        --------
        PoseStamped
            Navigation goal pose for the robot
        """
        # Extract face center and normal
        face_center = face_info['center']
        face_normal = face_info['normal']
        
        # 1. Create a pose in the camera frame
        approach_point = face_center - face_normal * approach_distance
        
        # Create a PoseStamped message in the camera frame
        pose_in_camera = PoseStamped()
        pose_in_camera.header.frame_id = camera_frame
        pose_in_camera.header.stamp = self.get_clock().now().to_msg()
        
        # Set the position
        pose_in_camera.pose.position.x = approach_point[0]
        pose_in_camera.pose.position.y = approach_point[1]
        pose_in_camera.pose.position.z = approach_point[2]
        
        # Calculate orientation to face the box
        # We want the robot to look at the face
        # For orientation, we need to compute a quaternion from the desired look direction
        # This is a simplified approach - we just make the robot face the box
        
        # First, we project the face normal onto the x-y plane (assuming z is up)
        # This gives us the direction in the horizontal plane
        horizontal_direction = np.array([face_normal[0], face_normal[1], 0])
        
        # Normalize the vector
        norm = np.linalg.norm(horizontal_direction)
        if norm < 1e-6:
            # If the normal is mostly vertical, use a default direction
            horizontal_direction = np.array([1, 0, 0])
        else:
            horizontal_direction = horizontal_direction / norm
        
        # Calculate the yaw angle from this direction
        yaw = math.atan2(horizontal_direction[1], horizontal_direction[0])
        
        # Convert to quaternion (simplified version just using yaw)
        pose_in_camera.pose.orientation.x = 0.0
        pose_in_camera.pose.orientation.y = 0.0
        pose_in_camera.pose.orientation.z = math.sin(yaw / 2)
        pose_in_camera.pose.orientation.w = math.cos(yaw / 2)
        
        # 2. Transform the pose to the robot's frame
        try:
            pose_in_robot_frame = self.tf_buffer.transform(pose_in_camera, robot_frame, rclpy.duration.Duration(seconds=1.0))
            return pose_in_robot_frame
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform pose: {e}')
            return None

    def navigate_to_face(self, face_info, camera_frame='xtion_rgb_optical_frame', robot_frame='base_footprint'):
        """
        Navigate the robot to the selected face.
        
        Parameters:
        -----------
        face_info : dict
            Dictionary containing face information
        camera_frame : str
            Name of the camera frame
        robot_frame : str
            Name of the robot's base frame
        
        Returns:
        --------
        bool
            True if navigation request was sent successfully, False otherwise
        """
        # Calculate the approach pose
        approach_pose = self.calculate_approach_pose(face_info, camera_frame, robot_frame)
        
        if approach_pose is None:
            self.get_logger().error('Failed to calculate approach pose')
            return False
        
        # Wait for the action server
        self.get_logger().info('Waiting for navigation action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Create the goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = approach_pose
        
        # Send the goal
        self.get_logger().info('Sending navigation goal')
        send_goal_future = self.nav_client.send_goal_async(nav_goal)
        
        # Add a callback to receive the result
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True
    
    def goal_response_callback(self, future):
        """
        Callback for the response to the navigation goal request.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        Callback for the navigation result.
        """
        result = future.result().result
        self.get_logger().info(f'Navigation finished with result: {result}')

def select_best_face_to_approach(faces, min_face_size=0.1):
    """
    Select the best face to approach based on various criteria.
    
    Parameters:
    -----------
    faces : list of dict
        List of face information as returned by detect_box_faces
    min_face_size : float
        Minimum face size to consider (square meters)
    
    Returns:
    --------
    dict or None
        Information about the selected face, or None if no suitable face found
    """
    if not faces:
        return None
    
    best_face = None
    best_score = float('-inf')
    
    for face in faces:
        # Skip faces that are too small
        points = face['points']
        if len(points) < 10:  # Arbitrary threshold
            continue
        
        # Estimate face size by computing the area of the convex hull
        # This is a simplification - you might want to use a more accurate approach
        # like projecting the points onto the plane and computing the area
        
        # Instead, let's use the variance of the points as a proxy for face size
        variance = np.var(points, axis=0)
        face_size_estimate = np.sqrt(variance[0]**2 + variance[1]**2 + variance[2]**2)
        
        if face_size_estimate < min_face_size:
            continue
        
        # Check if the face normal is pointing toward the camera
        # We prefer faces that are more facing the camera
        # Normal vector is assumed to point outward from the box
        normal = face['normal']
        center = face['center']
        
        # Calculate the angle between the normal and the vector from center to camera
        # The camera is at the origin of its own frame, so the vector is just -center
        vector_to_camera = -center
        vector_to_camera_norm = np.linalg.norm(vector_to_camera)
        
        if vector_to_camera_norm > 0:
            vector_to_camera = vector_to_camera / vector_to_camera_norm
            
            # Calculate the dot product (cos of the angle)
            dot_product = np.dot(normal, vector_to_camera)
            
            # Higher dot product means the face is more facing the camera
            # We want to approach faces that are more visible
            visibility_score = dot_product
        else:
            visibility_score = -1  # Default low score if the center is at the origin
        
        # Calculate the distance to the face
        distance = np.linalg.norm(center)
        
        # Combine factors into a score
        # We prefer faces that are:
        # 1. Larger
        # 2. More facing the camera
        # 3. Closer to the camera
        score = (face_size_estimate * 0.5) + (visibility_score * 0.3) + (1.0 / (distance + 0.1) * 0.2)
        
        if score > best_score:
            best_score = score
            best_face = face
    
    return best_face


def main(args=None):
    rclpy.init(args=args)
    node = BoxFaceNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
