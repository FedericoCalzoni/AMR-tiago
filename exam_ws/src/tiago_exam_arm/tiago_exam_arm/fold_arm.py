import rclpy, time, argparse
from rclpy.node import Node
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TiagoArmPositionNode(Node):
    def __init__(self, input_string):
        self.input_string = input_string
        super().__init__('tiago_arm_position')
        
        self.done_publisher = self.create_publisher(Bool, '/fold_arm/done', 10)
        # Publisher for arm controller
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # Define arm joint names for TIAGo
        self.arm_joint_names = [
            'arm_1_joint',  # 
            'arm_2_joint',  # 
            'arm_3_joint',  # Routa il braccio - ruota secondo joint
            'arm_4_joint',  # Piega il gomito (ultimo joint prima della parte bianca)
            'arm_5_joint',  # Ruota il polso
            'arm_6_joint',  # Piega il polso 
            'arm_7_joint'   # 
        ]
        
        time.sleep(1.0)

    def move_arm_to_position(self, positions, duration=2.0):
        """Send a command to move the arm to the specified positions"""
        if len(positions) != len(self.arm_joint_names):
            self.get_logger().error(f"Expected {len(self.arm_joint_names)} joint positions, got {len(positions)}")
            return False
            
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        
        trajectory.points = [point]
        self.publisher_.publish(trajectory)
        self.get_logger().info(f"Sent arm command: {positions}")
        return True

def main(args=None):
    
    # USAGE: ros2 run tiago_exam_arm fold_arm.py --input_string nav
    
    parser = argparse.ArgumentParser(description='ROS2 Shape Publisher Node')
    parser.add_argument('--input_string', type=str, default='arm',
                        help='Input string parameter')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = TiagoArmPositionNode(args.input_string)

    zero_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    folded_position_navigation = [0.0, 0.0, -3.0, 1.5, 0.0, 0.0, 0.0]
    folded_position_manipulation = [3.0, 0.0, -3.0, 1.5, 0.0, 0.0, 0.0]

    if args.input_string == "nav":
        folded_position = folded_position_navigation
    else:
        folded_position = folded_position_manipulation


    # node.move_arm_to_position(zero_position)
    # node.get_logger().info("Moving arm to zero position")
    # time.sleep(3.0)
    success = node.move_arm_to_position(folded_position)
    
    if success:
        node.get_logger().info("Command sent to fold arm")
        node.done_publisher.publish(Bool(data=True))
    else:
        node.get_logger().error("Failed to send arm command")
    
    # Wait a moment to ensure the command is sent
    time.sleep(1.0)
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()