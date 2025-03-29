import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Create a publisher to send joint trajectory commands
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Create a TF listener to receive the transform of the marker
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define robot joint names
        self.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Set a timer to check for the marker's transform periodically
        self.create_timer(1.0, self.update_arm_position)

    def update_arm_position(self):
        """Receives the transform from fake_transform and moves the arm towards the marker."""
        try:
            # Get the transform from the robot base to the marker frame
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',  # Robot's base frame
                'aruco_marker_frame',  # Marker frame from fake_transform
                rclpy.time.Time()
            )

            # Extract position data from the transform
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            self.get_logger().info(f"📍 Marker detected at: x={x:.2f}, y={y:.2f}, z={z:.2f}")

            # Convert position to joint angles (simplified for testing)
            joint_positions = self.calculate_joint_positions(x, y, z)

            # Move the arm to the calculated joint positions
            self.move_arm(joint_positions)

        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not get transform: {str(e)}")

    def calculate_joint_positions(self, x, y, z):
        """Simple function to calculate joint angles based on marker position."""
        # For testing, map the marker height (z) to the arm_3_joint
        arm_3_position = max(0.0, min(2.0, z))  # Keep value in a safe range

        return [0.0, 0.0, arm_3_position, 0.0, 0.0, 0.0, 0.0]  # Adjusted joint positions

    def move_arm(self, positions):
        """Moves the arm to the specified joint positions."""
        self.get_logger().info("🎯 Moving the arm to the marker position...")

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions  # Set calculated joint positions
        point.time_from_start.sec = 2  # Execution time

        msg.points.append(point)

        self.arm_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

