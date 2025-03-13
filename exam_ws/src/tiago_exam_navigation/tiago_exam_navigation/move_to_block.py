import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MoveToBlockNode(Node):
    def __init__(self):
        super().__init__('move_to_block_node')
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

def main(args=None):
    rclpy.init(args=args)
    node = MoveToBlockNode()

    # Esempio di invio di un obiettivo
    node.send_goal(1000, 1000)  # Usa i valori in pixel (che poi verranno convertiti in metri)
    
    rclpy.spin(node)  # Mantieni il nodo in esecuzione

    rclpy.shutdown()

if __name__ == '__main__':
    main()
