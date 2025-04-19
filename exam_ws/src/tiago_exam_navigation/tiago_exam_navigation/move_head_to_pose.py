#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

class MoveHead(Node):
    def __init__(self):
        super().__init__('move_head')
        
        # Publisher for trajectory commands
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/head_controller/joint_trajectory', 
            10
        )
    
    def move_head(self, pan, tilt):
        """Move the head to a specified position"""
        msg = JointTrajectory()
        msg.joint_names = ['head_1_joint', 'head_2_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start.sec = 1
        
        msg.points = [point]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving head to position: [{pan}, {tilt}]')


def main(args=None):
    rclpy.init(args=args)
    node = MoveHead()
    
    # Default position is "home" (0, 0)
    pan, tilt = 0.0, 0.0
    
    # Parse command line arguments if provided
    if len(sys.argv) > 2:
        try:
            pan = float(sys.argv[1])
            tilt = float(sys.argv[2])
        except ValueError:
            print("Error: Arguments must be numeric values")
            print("Usage: ros2 run <package_name> head_position_node.py [pan_value tilt_value]")
            return
    
    # Move the head
    node.move_head(pan, tilt)
    
    # Spin briefly to allow message publishing
    for i in range(5):  # Spin for ~0.5 seconds to ensure message is sent
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()