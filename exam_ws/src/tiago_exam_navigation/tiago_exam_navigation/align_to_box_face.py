import numpy as np
import math, cv2
import tf2_ros, rclpy
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
# $ source install/setup.bash
from std_msgs.msg import Bool
from tiago_interfaces.msg import BoxInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class BoxFaceNavigator(Node):
    def __init__(self):
        super().__init__('box_face_navigator')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Navigation client initialized. Waiting for server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Connected to navigation server!")
        
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.done_publisher = self.create_publisher(Bool, '/align_to_box_face/done', 10)
        self.movement_timer = self.create_timer(0.1, self.done_moving)
        self.create_subscription(BoxInfo, '/box/faces_info', self.face_callback, 10)
        self.create_subscription(Odometry, '/ground_truth_odom', self.odom_callback, 10)
        self.image_subscription = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.frame = None
        self.target_pose_received = False
        self.si_e_mosso = False
        self.done = False
        self.done_moving_= False
        self.linear_velocity = None
        self.angular_velocity = None
        self.shutdown_timer = None
        self.target_pose = None
        self.yaw_threshold = 5*np.pi/180  # 5 degrees
        self.image_saved = False
        self.navigation_in_progress = False

    def odom_callback(self, msg):
        # Extract velocities from the message
        self.linear_velocity = msg.twist.twist.linear
        if abs(self.linear_velocity.x) >= 0.1 or abs(self.linear_velocity.y) >= 0.1:
            self.si_e_mosso = True
            self.get_logger().info(f"Si e mosso: {self.si_e_mosso}")
        self.angular_velocity = msg.twist.twist.angular

    def done_moving(self):
        if self.linear_velocity is not None and self.angular_velocity is not None:
            if (abs(self.linear_velocity.x) <= 0.001 and abs(self.linear_velocity.y) <= 0.001 and abs(self.linear_velocity.z) <= 0.001 and
                abs(self.angular_velocity.z) <= 0.001):
                self.done_moving_ = True
        if self.done_moving_ and self.si_e_mosso:
            # Stop navigation to pose began in a previous step
            self.get_logger().info("\033[94mStopping navigation to pose\033[0m")
            self.nav_client._cancel_goal_async(self._goal_handle)
            self.done_moving_ = False
            self.si_e_mosso = False
            self.done = True
        else:
            self.done_moving_ = False
            self.done = False

    def is_done(self):
        if self.done:
            self.done_publisher.publish(Bool(data=True))

    def image_callback(self, msg):
        if self.frame is not None and not self.image_saved:
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

            self.image_saved = True
            self.status = 0
            #cv2.imwrite("camera_image.png", cv_image)

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
        if not self.target_pose_received:
            faces = []
            for i in range(2):
                face = msg.faces[i]
                face_info = {
                    'center': np.array([face.center[0], face.center[1], face.center[2]]),
                    'normal': np.array([face.normal[0], face.normal[1], face.normal[2]]),
                    'plane_coefficients': np.array([face.plane_coefficients[0], face.plane_coefficients[1], 
                                face.plane_coefficients[2], face.plane_coefficients[3]]),
                    'points': np.array([[point.x, point.y, point.z] for point in face.points])
                }
                faces.append(face_info) 

            self.target_pose_received = True
            # Choose best face
            best_face = self.select_best_face_to_approach(faces)
            # Process the face and navigate directly
            if best_face is not None:
                self.process_face_and_navigate(best_face)

    def process_face_and_navigate(self, face_info):
        """Process face info and navigate in one step."""
        if self.navigation_in_progress:
            return
        self.navigation_in_progress = True
        # Calculate face frame
        self.get_face_frame(face_info)
        # Calculate navigation goal
        target_pose = self.get_target_pose_from_face_frame()
        if target_pose is None:
            self.get_logger().error('Failed to calculate target pose')
            self.navigation_in_progress = False
            return
        # Validate navigation goal
        if not self.is_valid_pose(target_pose):
            self.get_logger().warn('Generated invalid navigation pose, skipping')
            self.navigation_in_progress = False
            return
        # Send navigation 
        self.send_navigation_goal(target_pose)   

    def get_face_frame(self, face_info): 
        face_center = face_info['center']
        face_normal = face_info['normal']
        # Define the z-axis as pointing upward
        z_axis = np.array([0, 0, 1])
        # Ensure the normal is a unit vector - x-axis
        x_axis = face_normal / np.linalg.norm(face_normal)
        # Calculate the y-axis as the cross product of z-axis and face normal
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        self.frame = [x_axis, y_axis, z_axis, face_center]

    def transform_face_to_map(self, face_info):
        try:
            # First check if we can do a direct transform
            if self.tf_buffer.can_transform(
                'map', 'head_front_camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            ):
                # Create a PoseStamped in the optical frame
                center_point = PoseStamped()
                center_point.header.frame_id = 'head_front_camera_depth_optical_frame'
                # Use current time
                center_point.header.stamp = rclpy.time.Time().to_msg()
                center_point.pose.position.x = float(face_info['center'][0])
                center_point.pose.position.y = float(face_info['center'][1])
                center_point.pose.position.z = float(face_info['center'][2])
                center_point.pose.orientation.w = 1.0  # Identity quaternion
                
                # Transform directly to map frame
                transformed_center = self.tf_buffer.transform(
                    center_point, 'map',
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                # Similarly transform a point along the normal
                normal_point = PoseStamped()
                normal_point.header.frame_id = 'head_front_camera_depth_optical_frame'
                normal_point.header.stamp = rclpy.time.Time().to_msg()
                normal_point.pose.position.x = float(face_info['center'][0] + face_info['normal'][0])
                normal_point.pose.position.y = float(face_info['center'][1] + face_info['normal'][1])
                normal_point.pose.position.z = float(face_info['center'][2] + face_info['normal'][2])
                normal_point.pose.orientation.w = 1.0  # Identity quaternion
                
                transformed_normal_point = self.tf_buffer.transform(
                    normal_point, 'map',
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                # Calculate the new normal vector in the map frame
                new_center = np.array([transformed_center.pose.position.x, 
                                    transformed_center.pose.position.y * 0.95, # Correction factor
                                    transformed_center.pose.position.z])
                new_normal_point = np.array([transformed_normal_point.pose.position.x, 
                                        transformed_normal_point.pose.position.y, 
                                        transformed_normal_point.pose.position.z])
                new_normal = new_normal_point - new_center
                
                # Normalize the new normal vector
                norm = np.linalg.norm(new_normal)
                if norm > 0:
                    new_normal = new_normal / norm
                else:
                    self.get_logger().warn('Normal vector has zero length after transformation')
                    return None
                
                # Create the transformed face info
                transformed_face_info = {
                    'center': new_center,
                    'normal': new_normal,
                    'plane_coefficients': face_info['plane_coefficients'],
                    'points': face_info['points']
                }
                
                #self.get_logger().info(f'Transformed face center from {face_info["center"]} to {new_center}')
                return transformed_face_info
            else:
                self.get_logger().warn('Cannot transform directly from optical frame to map')
                return None
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Direct transform failed: {e}. Trying alternative method...')
            transform = self.tf_buffer.lookup_transform(
                'map',
                'head_front_camera_depth_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Create stamped point
            p = tf2_geometry_msgs.PointStamped()
            p.header.frame_id = 'head_front_camera_depth_optical_frame'
            p.header.stamp = transform.header.stamp  # Use the same timestamp as the transform
            p.point.x = float(face_info['center'][0])
            p.point.y = float(face_info['center'][1])
            p.point.z = float(face_info['center'][2])
            
            # Transform point
            transformed_p = self.tf_buffer.transform(p, 'map')
            
            # Create normal point
            normal_p = tf2_geometry_msgs.PointStamped()
            normal_p.header.frame_id = 'head_front_camera_depth_optical_frame'
            normal_p.header.stamp = transform.header.stamp
            normal_p.point.x = float(face_info['center'][0] + face_info['normal'][0])
            normal_p.point.y = float(face_info['center'][1] + face_info['normal'][1])
            normal_p.point.z = float(face_info['center'][2] + face_info['normal'][2])
            
            # Transform normal point
            transformed_normal_p = self.tf_buffer.transform(normal_p, 'map')
            
            # Calculate new center and normal
            new_center = np.array([transformed_p.point.x, transformed_p.point.y, transformed_p.point.z])
            new_normal_point = np.array([transformed_normal_p.point.x, transformed_normal_p.point.y, transformed_normal_p.point.z])
            new_normal = new_normal_point - new_center
            
            # Normalize
            norm = np.linalg.norm(new_normal)
            if norm > 0:
                new_normal = new_normal / norm
            else:
                self.get_logger().warn('Normal vector has zero length after transformation')
                return None
            
            transformed_face_info = {
                'center': new_center,
                'normal': new_normal,
                'plane_coefficients': face_info['plane_coefficients'],
                'points': face_info['points']
            }
            return transformed_face_info

    def is_valid_pose(self, pose):
        x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        if x > 1.5 or abs(y) > 10.0 or abs(z) > 1.0:
            return False
        return True

    def send_navigation_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info(f'\033[92mSending navigation goal: [{pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z}]\033[0m')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        self.state_machine_timer = self.create_timer(0.1, self.is_done)

    def goal_response_callback(self, future):
        """Handle the goal response."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            self.navigation_in_progress = False
            return
        self.get_logger().info('Goal accepted by the action server')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result of the navigation action."""
        self.status = future.result().status
        if self.status == 4:
            self.get_logger().info('\033[94mNavigation goal succeeded\033[0m')
            self.shutdown_timer = self.create_timer(2.0, self.delayed_shutdown)
            self.done = True
        else:
            self.get_logger().warning(f'Navigation goal failed with status: {self.status}')
        self.navigation_in_progress = False

    def get_target_pose_from_face_frame(self):
        """Convert face frame to a target pose for navigation."""
        if not hasattr(self, 'frame') or self.frame is None:
            self.get_logger().error('Face frame not available')
            return None
        x_axis, y_axis, z_axis, face_center = self.frame
        #self.get_logger().info(f"x axis: {x_axis}")
        # The goal is 0.15 meters away from the face along its normal (x-axis)
        target_position = face_center + 0.15 * x_axis  # Move away from the face
        
        # Calculate yaw from the x_axis + check wheter they are aligned but pointing in opposite directions
        #self.get_logger().info(f"prima componente: {x_axis[0]}")
        #self.get_logger().info(f"seconda copmponente: {x_axis[0]}")
        # normal will be uscente dalla faccia, x axis will point toward the face -> default yaw = 180 ergo: np.pi - ...
        yaw = math.atan2(x_axis[1], x_axis[0])
        if yaw > 0:
            yaw = yaw - np.pi
        elif yaw < 0:
            yaw = yaw + np.pi
        if abs(yaw) < self.yaw_threshold:
            yaw = 0.0
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(target_position[0])
        pose.pose.position.y = float(target_position[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = yaw
        pose.pose.orientation.w = 1.0
        return pose

    def select_best_face_to_approach(self, faces):
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
        best_face = None
        best_score = float('-inf')
            
        for camera_face in faces:
            center = camera_face['center']    
            # Transform the face into the map referene frame
            transformed_face = self.transform_face_to_map(camera_face)
            if transformed_face is None:
                self.get_logger().warn('Could not transform face to map frame')
                self.navigation_in_progress = False
                return
            
            points = transformed_face['points']
            normal = transformed_face['normal']
                    
            # Let's use the variance of the points as a proxy for face size
            variance = np.var(points, axis=0)
            face_size_estimate = np.sqrt(variance[0]**2 + variance[1]**2 + variance[2]**2)        
                    
            # Calculate the distance between the camera and the center
            # The camera is at the origin of its own frame, so the vector is just -center
            vector_to_camera = -center
            distance = np.linalg.norm(vector_to_camera)
                  
            # Calculate the yaw required to align to the face
            yaw = math.atan2(normal[1], normal[0])
            if yaw > 0:
                yaw = yaw - np.pi
            elif yaw < 0:
                yaw = yaw + np.pi
                
            # Combine factors into a score
            # We prefer faces that are:
            # 1. More facing the Tiago
            # 2. Larger
            # 3. Closer to the camera
            score = (face_size_estimate * 0.5) + (yaw * 1.5) + (1.0 / (distance + 0.1) * 0.2)    
            if score > best_score:
                best_score = score
                best_face = transformed_face
            
        #self.get_logger().info(f"Best face is: {best_face}")
        return best_face

    def delayed_shutdown(self):
        self.get_logger().info("Delayed shutdown timer triggered - shutting down node")
        if self.shutdown_timer:
            self.shutdown_timer.cancel()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    node = BoxFaceNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()

if __name__ == '__main__':
    main()
