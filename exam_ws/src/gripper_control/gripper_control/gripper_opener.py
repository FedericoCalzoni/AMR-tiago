#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient


class GripperOpener(Node):
    def __init__(self):
        super().__init__('gripper_opener')
        self.client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.timer = self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available.')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.04, 0.04]  # [0.0, 0.0] for closed, [0.04, 0.04] for open
        point.time_from_start.sec = 1

        goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending goal to open gripper...')
        self.client.send_goal_async(goal_msg)
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = GripperOpener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
