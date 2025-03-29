"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros

class FakeTFPublisher(Node):
    def __init__(self):
        super().__init__('fake_tf_publisher')
        
        # Crea un broadcaster per pubblicare la trasformazione fittizia
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer per pubblicare periodicamente la trasformazione
        self.timer = self.create_timer(0.5, self.publish_fake_transform)

    def publish_fake_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"  # Frame di riferimento del robot
        t.child_frame_id = "aruco_marker_frame_approach"  # Frame fittizio dell'ArUco

        # Posizione fittizia del marker ArUco
        t.transform.translation.x = 0.5  # 50 cm davanti al robot
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.8  # 80 cm di altezza

        # Orientamento (quaternion unitario, nessuna rotazione)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Pubblica la trasformazione
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Pubblicata trasformazione fittizia: {t.child_frame_id}")

def main():
    rclpy.init()
    node = FakeTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
 """
 
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class FakeArucoMarker(Node):
    def __init__(self):
        super().__init__('fake_aruco_marker')

        # Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish TF every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_fake_transform)

        self.get_logger().info("✅ Fake ArUco Marker initialized!")

    def publish_fake_transform(self):
        """Publishes a fake transformation for the ArUco marker."""
        t = TransformStamped()

        # Set the timestamp
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"  # Parent frame (robot base)
        t.child_frame_id = "aruco_marker_frame"  # Fake marker frame

        # Fake position (moves in a small circle over time)
        time_now = self.get_clock().now().nanoseconds / 1e9
        t.transform.translation.x = -0.5 + 0.1 * math.sin(time_now)  # Moves left/right
        t.transform.translation.y = -0.2  # Fixed y position
        t.transform.translation.z = 1.0  # Fixed height

        # Fake orientation (no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"📡 Published fake TF: {t.transform.translation}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeArucoMarker()

    # Keep publishing the fake transform
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

