import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from PyKDL import Frame, Vector, Rotation
from std_msgs.msg import Bool


class ArucoGraspBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_grasp_pose_broadcaster')
        
        self.create_subscription(Bool, '/move_arm/done', self.move_arm_done_callback, 10)
        self.create_subscription(Bool, '/move_arm/get_frames', self.move_arm_get_frames_callback, 10)

        self.move_arm_done = False
        self.change_frame = True
        
        self.frame_target_history = []
        self.history_size = 5
        
        self.subscription = self.create_subscription(
            TransformStamped, 
            '/aruco_single/transform', 
            self.get_aruco_callback, 
            1
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.frame_approach = None
        self.frame_target = None
        self.timer = self.create_timer(0.1, self.publish_frames)
        
        # Create marker publisher for visualization
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/transform_arrows',
            10
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_aruco = self.create_timer(0.1, self.timer_tf_base)
        self.robot_base_frame = "base_link"
        self.camera_frame = "head_front_camera_rgb_optical_frame"
        self.frame_target_name = "aruco_marker_frame_target"
        self.frame_approach_name = "aruco_marker_frame_approach"
        self.frame_grasp_name = "aruco_marker_frame_grasp"
        
        self.t_base = None
        self.frame_aruco = None
        self.max_tilt = 0.01
        self.good_frame_target = None
        
        # Dictionary to store our transform pairs for visualization
        self.transform_pairs = {}
        
    def get_aruco_callback(self, msg):
        position = Vector(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z)
        orientation = Rotation.Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w) 
        # camera -> aruco
        self.frame_aruco = Frame(orientation, position)
        # self.get_logger().info(f"Received ArUco transform: pos={position}")
        # self.get_logger().info(f"quat=[x={msg.transform.rotation.x}, y={msg.transform.rotation.y}, z={msg.transform.rotation.z}, w={msg.transform.rotation.w}]")
        
        # Store camera -> aruco transform pair for visualization
        self.transform_pairs["camera_to_aruco"] = {
            "parent": self.camera_frame,
            "child": "aruco_marker",
            "frame": self.frame_aruco,
            "color": [1.0, 0.0, 0.0, 1.0]  # Red
        }
        
    def timer_tf_base(self):
        try:
            self.t_base = self.tf_buffer.lookup_transform(
                self.robot_base_frame, 
                self.camera_frame, 
                rclpy.time.Time()
            ) # camera -> base
            
            # Store base -> camera transform pair for visualization
            frame_robot = self.get_frame_kdl(self.t_base)
            self.transform_pairs["base_to_camera"] = {
                "parent": self.robot_base_frame,
                "child": self.camera_frame,
                "frame": frame_robot,
                "color": [0.0, 1.0, 0.0, 1.0]  # Green
            }
            
        except Exception as e:
            self.get_logger().warn(f'Could not transform base-camera! {str(e)}')
        return
        
    def get_frame_kdl(self, tf):
        pos = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
        quat = [tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w]
        frame = Frame(Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3]), Vector(pos[0], pos[1], pos[2]))
        return frame
        
    def publish_frame(self, frame, frame_name):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot_base_frame
        t.child_frame_id = frame_name
        
        # Apply 180 degree rotation about the x-axis
        rotation_90_x = Rotation.RPY(-np.pi/2, 0, 0)
        frame = frame * Frame(rotation_90_x)
        
        # position
        t.transform.translation.x = frame.p.x()
        t.transform.translation.y = frame.p.y()
        t.transform.translation.z = frame.p.z()
        
        quat = frame.M.GetQuaternion()
        quat = np.array(quat) / np.linalg.norm(quat)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        
    def publish_frames(self):
        if self.t_base is None or self.frame_aruco is None:
            return
            
        frame_robot = self.get_frame_kdl(self.t_base)
        

        # Calculate target frame (marker position in robot base frame)
        # frame_aruco: camera -> aruco
        # frame_robot: camera -> base
        frame_target = frame_robot * self.frame_aruco
        
        # Check if z-axis of frame_target is mostly aligned with z of world
        z_axis = frame_target.M.UnitZ()
        z_direction = abs(z_axis[2])
        
        self.publish_frame(frame_target, self.frame_target_name)
        
        if self.change_frame:
            # Check if the alignment is within the threshold
            if z_direction < (1-self.max_tilt):
                self.get_logger().warn(f"Frame not vertical:{str(z_direction)}")
                # Not aligned enough, don't publish frames
            else:                
                # Add current frame to history
                self.frame_target_history.append(frame_target)
                # Keep only the most recent frames
                if len(self.frame_target_history) > self.history_size:
                    self.frame_target_history.pop(0)
                self.good_frame_target = self.average_frames(self.frame_target_history)
            
            if self.good_frame_target is not None:
                frame_target = self.good_frame_target
            else:
                self.get_logger().warn("No good frame target found, not publishing.")
                return
        else:
            if self.good_frame_target is not None:
                frame_target = self.good_frame_target
            else:
                self.get_logger().warn("No good frame target found, not publishing.")
                return
            
                    
        
        # Calculate pre-grasp approach frame
        frame_approach = frame_target * Frame(Rotation(), Vector(0, 0, 0.5))
        # Rotate to have the gripper approach horizontally
        frame_approach.M.DoRotY(np.pi/2)
        # frame_approach.M.DoRotX(np.pi)
        self.publish_frame(frame_approach, self.frame_approach_name)
        
        # Calculate the final grasp frame
        # This is closer to the marker than the approach frame
        frame_grasp = frame_target * Frame(Rotation(), Vector(0, 0, 0.24))
        frame_grasp.M.DoRotY(np.pi/2)
        # frame_grasp.M.DoRotX(np.pi)
        self.publish_frame(frame_grasp, self.frame_grasp_name)

        # Publish visualization markers
        self.publish_transform_markers()
        
    def average_frames(self, frames):
        """
        Calculate the average of multiple KDL frames
        
        For position: Simple arithmetic mean of x, y, z components
        For orientation: Convert to quaternions, average, and convert back to rotation matrix
        """
        if not frames:
            return None
        
        # For position, simple average
        position = Vector(0, 0, 0)
        for frame in frames:
            position += frame.p
        position = position / len(frames)
        
        # For orientation, average quaternions
        import PyKDL
        quaternions = []
        for frame in frames:
            qx, qy, qz, qw = frame.M.GetQuaternion()
            quaternions.append((qx, qy, qz, qw))
        
        # Average quaternions
        avg_qx, avg_qy, avg_qz, avg_qw = self.average_quaternions(quaternions)
        
        # Create a rotation from the averaged quaternion
        rotation = Rotation.Quaternion(avg_qx, avg_qy, avg_qz, avg_qw)
        
        # Return a new frame with averaged position and orientation
        return Frame(rotation, position)

    def average_quaternions(self, quaternions):
        """
        Calculate the average of multiple quaternions
        
        Args:
            quaternions: List of (qx, qy, qz, qw) tuples
            
        Returns:
            Tuple of (qx, qy, qz, qw) representing the average quaternion
        """
        import numpy as np
        
        # Convert to numpy arrays for easier math
        q_array = np.array(quaternions)
        
        # Method 1: Simple arithmetic mean (works for quaternions that are close to each other)
        avg_q = np.mean(q_array, axis=0)
        
        # Normalize the result
        norm = np.linalg.norm(avg_q)
        if norm > 1e-10:  # Avoid division by zero
            avg_q = avg_q / norm
        
        return tuple(avg_q)
        
    def publish_transform_markers(self):
        """Create and publish arrow markers to visualize transforms"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Create a marker for each transform pair
        for key, transform_info in self.transform_pairs.items():
            parent_frame = transform_info["parent"]
            child_frame = transform_info["child"]
            frame = transform_info["frame"]
            color = transform_info["color"]
            
            # Create arrow marker
            marker = Marker()
            marker.header.frame_id = self.robot_base_frame  # Always use base_link as the reference
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "transform_arrows"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # We need to transform the start and end points to base_link coordinates
            try:
                if parent_frame != self.robot_base_frame:
                    # Get transform from parent_frame to base_link
                    parent_to_base = self.tf_buffer.lookup_transform(
                        self.robot_base_frame,
                        parent_frame,
                        rclpy.time.Time()
                    )
                    parent_to_base_frame = self.get_frame_kdl(parent_to_base)
                    
                    # Transform start point (origin of parent frame) to base_link
                    start_in_base = parent_to_base_frame
                    
                    # Transform end point (child position) to base_link
                    # Calculate the absolute position of child in parent frame
                    child_in_parent = frame
                    # Then transform to base frame
                    end_in_base = parent_to_base_frame * child_in_parent
                else:
                    # If parent is already base_link, no transform needed for start
                    start_in_base = Frame(Rotation(), Vector(0, 0, 0))
                    # End point is directly in base coordinates
                    end_in_base = frame
                
                # Create arrow points
                from geometry_msgs.msg import Point
                start = Point()
                start.x = start_in_base.p.x()
                start.y = start_in_base.p.y()
                start.z = start_in_base.p.z()
                
                end = Point()
                end.x = end_in_base.p.x()
                end.y = end_in_base.p.y()
                end.z = end_in_base.p.z()
                
                # Set arrow properties
                marker.scale.x = 0.01  # shaft diameter
                marker.scale.y = 0.02  # head diameter
                marker.scale.z = 0.1   # head length
                
                # Set arrow color
                marker.color = ColorRGBA()
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = color[3]
                
                # Add points to marker
                marker.points = [start, end]
                
                # Add to marker array
                marker_array.markers.append(marker)
                marker_id += 1
                
            except Exception as e:
                self.get_logger().warn(f'Could not transform for visualization: {str(e)}')
        
        # Publish the marker array
        self.marker_publisher.publish(marker_array)
        
    def move_arm_done_callback(self, msg):
        if msg.data == True:
            self.move_arm_done = True
            
    def move_arm_get_frames_callback(self, msg):
        if msg.data == True:
            self.change_frame = False

def main(args=None):
    rclpy.init(args=args)
    aruco_node = ArucoGraspBroadcaster()
    
    while not aruco_node.move_arm_done:
        rclpy.spin_once(aruco_node, timeout_sec=0.1)
        
    aruco_node.get_logger().info("Received move_arm/done message. Shutting down node.")
    
    # Clean up
    aruco_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()