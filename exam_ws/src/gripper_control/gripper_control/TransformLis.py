import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')

        # Crea un buffer per memorizzare le trasformazioni
        self.tf_buffer = Buffer()
        
        # Crea un TransformListener per ascoltare le trasformazioni
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Frame di riferimento
        self.robot_base_frame = "base_link"
        self.target_frame = "aruco_marker_frame_approach"

        # Crea un timer per controllare la trasformazione ogni 0.5 secondi
        self.timer = self.create_timer(0.5, self.lookup_transform)

    def lookup_transform(self):
        try:
            # Cerca la trasformazione tra il robot e il target
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.target_frame,
                rclpy.time.Time()
            )
            self.get_logger().info(f"Transform: {transform.transform.translation}")
        
        except Exception as e:
            self.get_logger().warn(f"Errore nel trovare la trasformazione: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TFListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

