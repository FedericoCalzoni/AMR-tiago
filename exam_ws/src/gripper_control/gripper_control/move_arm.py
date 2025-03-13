import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # Create a publisher to send joint trajectory commands to the arm
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def move_arm_to_pose(self, positions):
        """Moves the arm to the specified joint positions."""
        self.get_logger().info("🎯 Moving the arm to the target position...")

        # Create a JointTrajectory message
        msg = JointTrajectory()
        
        # Define the joint names of the robotic arm
        msg.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions  # Set the desired joint positions
        point.time_from_start.sec = 2  # Execution time for movement

        # Add the point to the trajectory message
        msg.points.append(point)

        # Publish the message to the arm controller
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()

    # Predefined position to pick up the cube
    target_positions = [0.0, -1.0, 0.5, 1.5, 0.0, 1.0, 0.0]
    # Inverse position to place the cube
    #target_positions = [0.5, 0.0, 0.5, 1.5, 0.0, 1.0, 0.0]
    #target_positions = [0.5, 2.5, 0.5, 1.0, 0.0, 1.0, 5.0]
    # Move the arm to the target position
    node.move_arm_to_pose(target_positions)

    # Allow some time for the movement to execute
    rclpy.spin_once(node, timeout_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
