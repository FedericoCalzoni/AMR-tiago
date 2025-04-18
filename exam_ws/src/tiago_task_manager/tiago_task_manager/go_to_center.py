# go_to_center.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoToCenter(Node):
    def __init__(self):
        super().__init__('go_to_center')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'  # oppure 'odom', dipende da dove hai le coordinate
        goal.header.stamp = self.get_clock().now().to_msg()

        # 💡 Setta qui le tue coordinate del centro
        goal.pose.position.x = 6.0
        goal.pose.position.y = 2.4
        goal.pose.orientation.w = 0.50  # Orientamento neutro

        self.get_logger().info('📍 Sending center goal...')
        self.publisher.publish(goal)
        self.timer.cancel()  # Inviato una volta sola
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GoToCenter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
