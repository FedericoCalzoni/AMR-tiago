from geometry_msgs.msg import PoseStamped

self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

def send_goal(self, x, y):
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = self.get_clock().now().to_msg()
    goal.pose.position.x = x / 100  # Converti pixel in metri
    goal.pose.position.y = y / 100
    goal.pose.orientation.w = 1.0

    self.goal_pub.publish(goal)
    self.get_logger().info(f"📍 Obiettivo inviato a: x={goal.pose.position.x}, y={goal.pose.position.y}")
    