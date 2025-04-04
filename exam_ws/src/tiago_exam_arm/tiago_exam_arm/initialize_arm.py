import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Define the joint names of the robotic arm
        self.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Create a publisher to send joint trajectory commands to the arm
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # TF Listener to receive the transformation from the ArUco marker
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the transformation updates at a fixed rate
        self.timer = self.create_timer(1.0, self.update_arm_target)

        self.get_logger().info("✅ ArmController node initialized!")

    def update_arm_target(self):
        """Updates the arm target position based on the marker's transformation."""
        try:
            # Get the transformation from the robot's base to the marker frame
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link', 'aruco_marker_frame', rclpy.time.Time()
            )

            # Extract the translation coordinates (x, y, z)
            x, y, z = transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z

            # Convert the (x, y, z) position into joint angles (This requires IK or pre-defined mapping)
            joint_positions = self.calculate_joint_positions(x, y, z)

            # Move the arm to the computed joint positions
            self.move_arm_to_pose(joint_positions)

        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not get transform: {e}")

    def calculate_joint_positions(self, x, y, z):
        """Converts the ArUco marker position into joint angles. Placeholder function."""
        # Placeholder: Replace with an inverse kinematics (IK) solver
        self.get_logger().info(f"🔄 Converting (x={x}, y={y}, z={z}) to joint positions...")

        return [0.0, -1.0, 0.5, 1.5, 0.0, 1.0, 0.0]  # Example static position

    def move_arm_to_pose(self, positions):
        """Sends a command to move the arm to the specified joint positions."""
        self.get_logger().info(f"🎯 Moving arm to: {positions}")

        # Create a JointTrajectory message
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        # Define a trajectory point with the desired joint positions
        point = JointTrajectoryPoint()
        point.positions = positions  
        point.time_from_start.sec = 2  # Movement duration in seconds

        # Add the point to the trajectory message
        msg.points.append(point)

        # Publish the message to the arm controller
        self.arm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()

    # Spin to keep receiving transformations and updating the arm
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

