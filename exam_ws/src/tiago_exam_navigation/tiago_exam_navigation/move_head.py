import rclpy
from rclpy.node import Node
import sys, select, termios, tty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Define the keys for controlling the head
move_bindings = {
    'a': (0.1, 0),  # Up
    'd': (-0.1, 0), # Down
    'w': (0, 0.1),  # Left
    's': (0, -0.1)  # Right
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [])
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class HeadTeleopNode(Node):
    def __init__(self):
        super().__init__('tiago_head_teleop')
        self.publisher_ = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.head_state = JointTrajectory()
        self.head_state.joint_names = ['head_1_joint', 'head_2_joint']
        self.current_position = [0.0, 0.0]  # Initialize head position
        self.settings = termios.tcgetattr(sys.stdin)
        print("Use 'w', 's', 'a', 'd' keys to move the head. Press 'q' to quit.")
    
    def timer_callback(self):
        key = get_key(self.settings)
        if key in move_bindings.keys():
            self.get_logger().info(f"Key pressed: {key}")
            self.current_position[0] += move_bindings[key][0]
            self.current_position[1] += move_bindings[key][1]
            
            point = JointTrajectoryPoint()
            point.positions = self.current_position
            point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
            
            self.head_state.points = [point]
            self.publisher_.publish(self.head_state)
        elif key == 'q':
            rclpy.shutdown()
        self.publisher_.publish(self.head_state)

def main(args=None):
    rclpy.init(args=args)
    node = HeadTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()