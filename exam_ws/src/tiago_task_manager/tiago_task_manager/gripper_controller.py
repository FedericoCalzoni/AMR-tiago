#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from std_msgs.msg import Bool

class GripperCloser(Node):
    def __init__(self):
        super().__init__('gripper_closer')
        self.client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        
        # Publisher per notificare quando l'operazione è completata
        self.completion_publisher = self.create_publisher(Bool, '/gripper_close_completed', 10)
        
        self.timer = self.create_timer(1.0, self.send_goal)
    
    def send_goal(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available.')
            return
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]  # [0.0, 0.0] for closed, [0.04, 0.04] for open
        point.time_from_start.sec = 1
        goal_msg.trajectory.points.append(point)
        
        self.get_logger().info('Sending goal to close gripper...')
        
        # Invia il goal e ottiene il futuro per monitorare il completamento
        self.send_goal_future = self.client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.timer.cancel()
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        # Ottieni il risultato futuro
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        self.get_logger().info('Gripper closing completed')
        
        # Notifica completamento
        completion_msg = Bool()
        completion_msg.data = True
        self.completion_publisher.publish(completion_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperCloser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()