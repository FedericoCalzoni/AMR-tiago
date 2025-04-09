#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np

class CubeCornersPublisher(Node):
    """
    ROS2 node that publishes the 8 corners of a cube.
    Takes the cube's center coordinates and edge length as parameters.
    """
    
    def __init__(self):
        super().__init__('cube_corners_publisher')
        
        # Declare parameters with respect
        x = 1.15 - 0.5
        self.declare_parameter('cube_center_x', x)
        y = -4.499995 + 2.9 + 1.3
        self.declare_parameter('cube_center_y', y)
        z = 0.333789 
        self.declare_parameter('cube_center_z', z)
        self.declare_parameter('cube_edge_length', 0.07)
        self.declare_parameter('publish_frequency', 30.0)  # Hz
        
        # Get parameter values
        self.center_x = self.get_parameter('cube_center_x').value
        self.center_y = self.get_parameter('cube_center_y').value
        self.center_z = self.get_parameter('cube_center_z').value
        self.edge_length = self.get_parameter('cube_edge_length').value
        self.frequency = self.get_parameter('publish_frequency').value
        
        # Create publishers
        self.corners_pub = self.create_publisher( MarkerArray, 'cube_corners', 10)
        self.center_pub = self.create_publisher( PointStamped, 'cube_center', 10)
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_corners)
        
        self.get_logger().info('Cube Corners Publisher initialized')
        self.get_logger().info(f'Cube center: ({self.center_x}, {self.center_y}, {self.center_z})')
        self.get_logger().info(f'Cube edge length: {self.edge_length}')
        
    def calculate_corners(self):
        """Calculate the 8 corners of the cube."""
        half_edge = self.edge_length / 2.0
        center = np.array([self.center_x, self.center_y, self.center_z])
        
        # Define the relative positions of corners from the center
        # Each corner is half an edge length away from the center in each dimension
        offsets = np.array([
            [ half_edge,  half_edge,  half_edge],  # 0: front top right
            [ half_edge,  half_edge, -half_edge],  # 1: back top right
            [ half_edge, -half_edge,  half_edge],  # 2: front bottom right
            [ half_edge, -half_edge, -half_edge],  # 3: back bottom right
            [-half_edge,  half_edge,  half_edge],  # 4: front top left
            [-half_edge,  half_edge, -half_edge],  # 5: back top left
            [-half_edge, -half_edge,  half_edge],  # 6: front bottom left
            [-half_edge, -half_edge, -half_edge],  # 7: back bottom left
        ])
        
        # Calculate absolute positions
        corners = center + offsets
        return corners
        
    def publish_corners(self):
        """Publish the cube corners as markers and the center as a point."""
        corners = self.calculate_corners()
        
        # Create MarkerArray for corners
        marker_array = MarkerArray()
        for i, corner in enumerate(corners):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cube_corners"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = corner[0]
            marker.pose.position.y = corner[1]
            marker.pose.position.z = corner[2]
            
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        # Create lines between corners to visualize cube edges
        edges = [
            (0, 1), (0, 2), (0, 4),
            (1, 3), (1, 5),
            (2, 3), (2, 6),
            (3, 7),
            (4, 5), (4, 6),
            (5, 7),
            (6, 7)
        ]
        
        for i, (start_idx, end_idx) in enumerate(edges):
            line_marker = Marker()
            line_marker.header.frame_id = "base_link"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "cube_edges"
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # Add start and end points to the line
            start = Point(x=corners[start_idx][0], y=corners[start_idx][1], z=corners[start_idx][2])
            end = Point(x=corners[end_idx][0], y=corners[end_idx][1], z=corners[end_idx][2])
            line_marker.points = [start, end]
            
            line_marker.scale.x = 0.005  # Line width
            line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            
            marker_array.markers.append(line_marker)
        
        # Also add a marker for the center
        center_marker = Marker()
        center_marker.header.frame_id = "base_link"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "cube_center"
        center_marker.id = 0
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = self.center_x
        center_marker.pose.position.y = self.center_y
        center_marker.pose.position.z = self.center_z
        center_marker.pose.orientation.w = 1.0
        center_marker.scale.x = 0.015
        center_marker.scale.y = 0.015
        center_marker.scale.z = 0.015
        center_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue center
        
        marker_array.markers.append(center_marker)
        
        # Publish all markers
        self.corners_pub.publish(marker_array)
        
        # Publish center point
        center_point = PointStamped()
        center_point.header.frame_id = "base_link"
        center_point.header.stamp = self.get_clock().now().to_msg()
        center_point.point.x = self.center_x
        center_point.point.y = self.center_y
        center_point.point.z = self.center_z
        
        self.center_pub.publish(center_point)
        
        self.get_logger().debug(f'Published cube corners and center')

def main(args=None):
    rclpy.init(args=args)
    node = CubeCornersPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()