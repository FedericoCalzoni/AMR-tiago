import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/command', 10)
        
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.gripper_state = JointTrajectory()
        self.gripper_state.joint_names = ['gripper_finger_joint']
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.moving_down = True

    def timer_callback(self):
        self.pick_object()
        #if self.moving_down:
        #    self.move_arm_down()
        #else:
        #    self.move_arm_up()

    def move_arm_down(self):
        traj = JointTrajectory()
        traj.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example positions for moving down
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.arm_publisher.publish(traj)
        self.moving_down = False
        self.get_logger().info('Moving arm down')

    def move_arm_up(self):
        traj = JointTrajectory()
        traj.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example positions for moving up
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.arm_publisher.publish(traj)
        self.moving_down = True
        self.get_logger().info('Moving arm up')

    def pick_object(self):
        point = JointTrajectoryPoint()
        point.positions = [0.0]  # Close gripper
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        self.gripper_state.points = [point]
        self.gripper_publisher.publish(self.gripper_state)
        self.get_logger().info('Closing gripper')

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()