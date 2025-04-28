import rclpy
from rclpy.node import Node
import sys, select, termios, tty, os
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Define the keys for controlling the head
move_bindings = {
    'a': (0.1, 0),  # Left
    'd': (-0.1, 0),  # Right
    'w': (0, 0.1),  # Up
    's': (0, -0.1)  # Down
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Fixed unpacking
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class HeadTeleopNode(Node):
    def __init__(self):
        super().__init__('tiago_head_teleop')
        self.publisher_ = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(
            Image, 
            '/head_front_camera/rgb/image_raw', 
            self.image_callback, 
            10
        )
        self.bridge = CvBridge()
        self.img_msg = None
        self.img = None
        self.head_state = JointTrajectory()
        self.head_state.joint_names = ['head_1_joint', 'head_2_joint']
        self.current_position = [0.0, 0.0]  # Initialize head position
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Create a timer with faster rate for more responsive controls
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # Create the window ahead of time
        cv2.namedWindow("Camera Image", cv2.WINDOW_NORMAL)
        
        print("Use 'w', 's', 'a', 'd' keys to move the head. Press 'q' to quit.")

    def image_callback(self, msg):
        """Callback for image subscription."""
        try:
            self.img_msg = msg
            self.img = self.bridge.imgmsg_to_cv2(self.img_msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def timer_callback(self):
        """Timer callback to handle key presses and image display."""
        # Display the image if available
        if self.img is not None:
            try:
                cv2.imshow("Camera Image", self.img)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"Error displaying image: {e}")
        
        # Get keyboard input
        key = get_key(self.settings)
        
        if key in move_bindings.keys():
            self.get_logger().info(f"Key pressed: {key}")
            self.current_position[0] += move_bindings[key][0]
            self.current_position[1] += move_bindings[key][1]
            
            # Create and publish trajectory
            point = JointTrajectoryPoint()
            point.positions = self.current_position
            point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
            self.head_state.points = [point]
            self.publisher_.publish(self.head_state)
            
        elif key == 'q':
            # Clean shutdown
            self.get_logger().info('Shutting down...')
            
            # Close OpenCV windows
            cv2.destroyAllWindows()
            
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            # Exit
            os._exit(0)

def main(args=None):
    # Save original terminal settings
    original_settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = HeadTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()