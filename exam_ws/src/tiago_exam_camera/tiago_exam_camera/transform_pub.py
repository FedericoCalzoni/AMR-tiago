import rclpy, tf2_ros, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped

#!/usr/bin/env python

import cv2.aruco as aruco

class ArucoTransformPublisher(Node):
    def __init__(self):
        super().__init__('aruco_transform_publisher')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self.image_callback, 10)
        self.aruco_publisher = self.create_publisher(TransformStamped, '/aruco_single/transform', 10)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

    def image_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # Process the image and publish the transformation
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            for corner, id in zip(corners, ids):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corner, 0.05, self.camera_matrix, self.dist_coeffs)
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'camera_frame'
                transform.child_frame_id = f'aruco_marker_{id[0]}'
                transform.transform.translation.x = tvec[0][0][0]
                transform.transform.translation.y = tvec[0][0][1]
                transform.transform.translation.z = tvec[0][0][2]
                transform.transform.rotation = self.rvec_to_quaternion(rvec)
                
                self.tf_broadcaster.sendTransform(transform)

    def rvec_to_quaternion(self, rvec):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quaternion = tf_transformations.quaternion_from_matrix(rotation_matrix)
        return quaternion

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()