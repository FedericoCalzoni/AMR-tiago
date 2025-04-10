#!/usr/bin/env python3

import rclpy, argparse
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point, Quaternion, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
from math import sin, cos, atan2, sqrt
import tf2_ros
from tf2_ros import TransformBroadcaster

class ShapePublisher(Node):
    """
    ROS2 node that publishes corners of either a cube or a parallelepiped,
    with proper coordinate transformation.
    - For 'aruco' input: publishes a cube
    - For other inputs: publishes a parallelepiped with square base (0.5x0.5) and height 0.3
    """
    
    def __init__(self, input_string):
        super().__init__('shape_publisher')
        
        self.input_string = input_string
        self.get_logger().info(f'Received string: {self.input_string}')

        # Robot information (spawn position and orientation)
        self.robot_x = 0.520218  # Robot spawn position X 
        self.robot_y = -4.202210 # Robot spawn position Y  
        self.robot_z = 0.0  # Robot spawn position Z 
        self.robot_yaw = -0.111532  # Robot yaw rotation in radians
        
        # Set up tf2 listener and broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Define shape properties based on input string
        if self.input_string == 'aruco':
            self.center_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)   # Blue
            self.edges_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)    # Green
            # Shape properties (world coordinates)
            self.world_x = 1.15  # World X coordinate for the cube 
            self.world_y = -4.499995 # World Y coordinate for the cube 
            self.world_z = 0.333783   # World Z coordinate for the cube
            self.x_length = 0.07  # cube edge length 
            self.y_length = 0.07  # cube edge length
            self.z_length = 0.07  # cube edge length
            # Create publishers
            self.corners_pub = self.create_publisher(MarkerArray, '/Aruco_corners', 10)
            self.center_pub = self.create_publisher(PointStamped, '/Aruco_center', 10)
        else:
            self.edges_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)    # Blue
            self.center_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)   # Green
            # Shape properties (world coordinates)
            self.world_x = 1.2  # World X coordinate for the parallelepiped
            self.world_y = -4.5 # World Y coordinate for the parallelepiped
            self.world_z = 0.15  # World Z coordinate for the parallelepiped
            self.x_length = 0.5  # square base side length
            self.y_length = 0.5  # square base side length
            self.z_length = 0.3  # height
            # Create publishers
            self.corners_pub = self.create_publisher(MarkerArray, '/Box_corners', 10)
            self.center_pub = self.create_publisher(PointStamped, '/Box_center', 10)
        
        # Calculate the transformed coordinates (world to robot frame)
        self.robot_frame_coords = self.transform_world_to_robot_frame(
            self.world_x, self.world_y, self.world_z
        )
        
        # Create timer for publishing
        self.declare_parameter('publish_frequency', 30.0)  # Hz
        self.frequency = self.get_parameter('publish_frequency').value
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_shape)
        
        # Log information
        shape_type = "cube" if self.input_string == 'aruco' else "parallelepiped"
        self.get_logger().info(f'{shape_type.capitalize()} Publisher initialized')
        self.get_logger().info(f'Robot position (world): ({self.robot_x}, {self.robot_y}, {self.robot_z})')
        self.get_logger().info(f'Robot yaw: {self.robot_yaw} radians')
        self.get_logger().info(f'{shape_type.capitalize()} position (world): ({self.world_x}, {self.world_y}, {self.world_z})')
        self.get_logger().info(f'{shape_type.capitalize()} dimensions: {self.x_length} x {self.y_length} x {self.z_length}')
        self.get_logger().info(f'{shape_type.capitalize()} position (robot frame): ({self.robot_frame_coords[0]}, {self.robot_frame_coords[1]}, {self.robot_frame_coords[2]})')
    
    def transform_world_to_robot_frame(self, world_x, world_y, world_z):
        """
        Transform coordinates from world frame to robot frame.
        Takes into account robot position and orientation.
        """
        # Get relative position from robot
        dx = world_x - self.robot_x
        dy = world_y - self.robot_y
        dz = world_z - self.robot_z
        
        # Apply rotation (around Z axis) to get coordinates in robot frame
        # For world -> robot frame transformation, we use the negative of robot's yaw
        robot_frame_x = dx * cos(self.robot_yaw) + dy * sin(self.robot_yaw)
        robot_frame_y = -dx * sin(self.robot_yaw) + dy * cos(self.robot_yaw)
        robot_frame_z = dz
        
        return robot_frame_x, robot_frame_y, robot_frame_z
    
    def broadcast_transform(self):
        """Broadcast the transform from base_link to shape frame."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "shape_frame"
        
        # Set the translation
        t.transform.translation.x = self.robot_frame_coords[0]
        t.transform.translation.y = self.robot_frame_coords[1]
        t.transform.translation.z = self.robot_frame_coords[2]
        
        # Set the rotation (need to preserve the yaw from world frame to shape frame)
        # If box has no yaw wrt world and robot has yaw, the box will have -robot_yaw wrt robot
        q = self.euler_to_quaternion(0, 0, -self.robot_yaw)
        t.transform.rotation = q
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        
        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr
        
        return Quaternion(x=x, y=y, z=z, w=w)
        
    def publish_shape(self):
        """Publish the shape as a marker array with proper coordinates."""
        # Broadcast the transform first
        self.broadcast_transform()
        
        # Create MarkerArray
        marker_array = MarkerArray()
        
        # Create shape marker
        shape_marker = Marker()
        shape_marker.header.frame_id = "base_link"
        shape_marker.header.stamp = self.get_clock().now().to_msg()
        shape_marker.ns = "shape_volume"
        shape_marker.id = 0
        shape_marker.type = Marker.CUBE
        shape_marker.action = Marker.ADD
        
        # Set position in robot frame coordinates
        shape_marker.pose.position.x = self.robot_frame_coords[0]
        shape_marker.pose.position.y = self.robot_frame_coords[1]
        shape_marker.pose.position.z = self.robot_frame_coords[2]
        
        # Set orientation - need to account for the yaw difference
        q = self.euler_to_quaternion(0, 0, -self.robot_yaw)
        shape_marker.pose.orientation = q
        
        # Set dimensions
        shape_marker.scale.x = self.x_length
        shape_marker.scale.y = self.y_length
        shape_marker.scale.z = self.z_length
        
        # Set color (semi-transparent)
        shape_color = ColorRGBA(r=0.2, g=0.5, b=0.7, a=0.6)
        shape_marker.color = shape_color
        
        marker_array.markers.append(shape_marker)
        
        # Add center marker
        center_marker = Marker()
        center_marker.header.frame_id = "base_link"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "shape_center"
        center_marker.id = 0
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = self.robot_frame_coords[0]
        center_marker.pose.position.y = self.robot_frame_coords[1]
        center_marker.pose.position.z = self.robot_frame_coords[2]
        center_marker.pose.orientation.w = 1.0
        center_marker.scale.x = 0.02
        center_marker.scale.y = 0.02
        center_marker.scale.z = 0.02
        center_marker.color = self.center_color
        
        marker_array.markers.append(center_marker)
        
        # Add wireframe edges (to better visualize the shape)
        self.add_wireframe_to_markers(marker_array)
        self.corners_pub.publish(marker_array)

        # Also publish center point as a PointStamped message
        center_point = PointStamped()
        center_point.header.frame_id = "base_link"
        center_point.header.stamp = self.get_clock().now().to_msg()
        center_point.point.x = self.robot_frame_coords[0]
        center_point.point.y = self.robot_frame_coords[1]
        center_point.point.z = self.robot_frame_coords[2]
        self.center_pub.publish(center_point)
        
    def add_wireframe_to_markers(self, marker_array):
        """Add wireframe edges to the marker array to better visualize the shape."""
        # Calculate corners in local coordinates
        half_x = self.x_length / 2.0
        half_y = self.y_length / 2.0
        half_z = self.z_length / 2.0
        
        corners_local = np.array([
            [ half_x,  half_y,  half_z],  # 0: front top right
            [ half_x,  half_y, -half_z],  # 1: back top right
            [ half_x, -half_y,  half_z],  # 2: front bottom right
            [ half_x, -half_y, -half_z],  # 3: back bottom right
            [-half_x,  half_y,  half_z],  # 4: front top left
            [-half_x,  half_y, -half_z],  # 5: back top left
            [-half_x, -half_y,  half_z],  # 6: front bottom left
            [-half_x, -half_y, -half_z],  # 7: back bottom left
        ])
        
        # Define edges
        edges = [
            (0, 1), (0, 2), (0, 4),
            (1, 3), (1, 5),
            (2, 3), (2, 6),
            (3, 7),
            (4, 5), (4, 6),
            (5, 7),
            (6, 7)
        ]
        
        # Rotation matrix for the corners (to match the orientation of the shape)
        rotation_matrix = np.array([
            [cos(-self.robot_yaw), -sin(-self.robot_yaw), 0],
            [sin(-self.robot_yaw), cos(-self.robot_yaw), 0],
            [0, 0, 1]
        ])
        
        # Rotate corners
        corners_rotated = np.array([np.dot(rotation_matrix, corner) for corner in corners_local])
        
        # Create line markers for edges
        for i, (start_idx, end_idx) in enumerate(edges):
            line_marker = Marker()
            line_marker.header.frame_id = "base_link"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "shape_edges"
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # Add start and end points to the line
            start_point = Point(
                x=self.robot_frame_coords[0] + corners_rotated[start_idx][0],
                y=self.robot_frame_coords[1] + corners_rotated[start_idx][1],
                z=self.robot_frame_coords[2] + corners_rotated[start_idx][2]
            )
            
            end_point = Point(
                x=self.robot_frame_coords[0] + corners_rotated[end_idx][0],
                y=self.robot_frame_coords[1] + corners_rotated[end_idx][1],
                z=self.robot_frame_coords[2] + corners_rotated[end_idx][2]
            )
            
            line_marker.points = [start_point, end_point]
            line_marker.scale.x = 0.015  # Line width
            line_marker.color = self.edges_color
            
            marker_array.markers.append(line_marker)

def main(args=None):
    parser = argparse.ArgumentParser(description='ROS2 Shape Publisher Node')
    parser.add_argument('--input_string', type=str, default='aruco',
                        help='Input string parameter')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ShapePublisher(args.input_string)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()