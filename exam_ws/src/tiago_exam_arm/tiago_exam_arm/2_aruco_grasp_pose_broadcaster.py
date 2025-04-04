import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from PyKDL import Frame, Vector, Rotation

class ArucoGraspBroadcaster(Node):

    def __init__(self):
        super().__init__('aruco_subscriber')
        
        self.subscription = self.create_subscription(TransformStamped, '/aruco_single/transform', self.get_aruco_callback, 1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.frame_approach = None
        self.frame_target = None

        self.timer = self.create_timer(0.1, self.publish_frames)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_aruco = self.create_timer(0.1, self.timer_tf_base)

        self.robot_base_frame = "base_link"
        self.camera_frame = "head_front_camera_rgb_optical_frame"
        self.frame_target_name = "aruco_marker_frame_target"
        self.frame_approach_name = "aruco_marker_frame_approach"
        self.frame_grasp_name = "aruco_marker_frame_grasp"
        
        self.t_base = None
        self.frame_aruco = None

    def get_aruco_callback(self, msg):
        position = Vector(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z)
        orientation = Rotation.Quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w) 
        self.frame_aruco = Frame(orientation, position)
        self.get_logger().info(f"Received ArUco transform: pos={position}")
        self.get_logger().info(f"quat=[x={msg.transform.rotation.x}, y={msg.transform.rotation.y}, z={msg.transform.rotation.z}, w={msg.transform.rotation.w}]")

    def timer_tf_base(self):
        try:
            self.t_base = self.tf_buffer.lookup_transform(self.robot_base_frame, self.camera_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Could not transform base-camera! {str(e)}')
        return

    def get_frame_kdl(self, tf):
        pos = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
        quat = [tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w]

        frame = Frame(Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3]), Vector(pos[0], pos[1], pos[2]))
        return frame


    def publish_frame(self, frame, frame_name):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot_base_frame
        t.child_frame_id = frame_name

        # position
        t.transform.translation.x = frame.p.x()
        t.transform.translation.y = frame.p.y()
        t.transform.translation.z = frame.p.z()
                
        quat = frame.M.GetQuaternion()
        quat = np.array(quat) / np.linalg.norm(quat)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        
    def publish_frames(self):
        if self.t_base is None or self.frame_aruco is None:
            return

        frame_robot = self.get_frame_kdl(self.t_base)
        
        # Calculate target frame (marker position in robot base frame)
        frame_target = frame_robot * self.frame_aruco
        self.publish_frame(frame_target, self.frame_target_name)
        
        # Calculate pre-grasp approach frame (offset in front of the marker)
        # Creating an approach position 10cm in front of the marker
        frame_approach = frame_target * Frame(Rotation(), Vector(0, 0, 0.5))
        # Rotate to have the gripper approach horizontally
        frame_approach.M.DoRotY(np.pi/2)
        self.publish_frame(frame_approach, self.frame_approach_name)
        
        # Calculate the final grasp frame
        # This is closer to the marker than the approach frame
        frame_grasp = frame_target * Frame(Rotation(), Vector(0, 0, 0.2))
        frame_grasp.M.DoRotY(np.pi/2)
        self.publish_frame(frame_grasp, self.frame_grasp_name)

def main(args=None):
    rclpy.init(args=args)

    aruco_node = ArucoGraspBroadcaster()

    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()