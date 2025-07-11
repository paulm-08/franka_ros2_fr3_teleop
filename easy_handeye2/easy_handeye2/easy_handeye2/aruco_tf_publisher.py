#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

from transformations import quaternion_from_matrix
from scipy.spatial.transform import Rotation as R


class ArucoTfPublisher(Node):
    def __init__(self):
        super().__init__('aruco_tf_publisher')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.10)  # in meters
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('tracking_marker_frame', 'aruco_marker_frame')

        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.tracking_marker_frame = self.get_parameter('tracking_marker_frame').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.camera_matrix = None
        self.dist_coeffs = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.image_pub = self.create_publisher(Image, '/aruco/annotated_image', 10)

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera info received and stored.")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
        parameters = cv2.aruco.DetectorParameters()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.marker_id:
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs
                    )

                    # Create the coordinate transform
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = self.camera_frame
                    t.child_frame_id = self.tracking_marker_frame

                    # Store the translation (i.e. position) information
                    t.transform.translation.x = tvec[0][0][0]
                    t.transform.translation.y = tvec[0][0][1]
                    t.transform.translation.z = tvec[0][0][2]

                    # Store the rotation information
                    rotation_matrix = np.eye(4)
                    rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[0][0]))[0]
                    r = R.from_matrix(rotation_matrix[0:3, 0:3])
                    quat = r.as_quat()

                    # Quaternion format     
                    t.transform.rotation.x = quat[0] 
                    t.transform.rotation.y = quat[1] 
                    t.transform.rotation.z = quat[2] 
                    t.transform.rotation.w = quat[3]

                    # Send the transform
                    self.tf_broadcaster.sendTransform(t) 

                    # Draw marker axes on the image for visualization
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], self.marker_size * 0.5)
                    annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    annotated_msg.header.stamp = msg.header.stamp
                    self.image_pub.publish(annotated_msg)

                    # self.get_logger().info(f"Published TF for marker {marker_id}")
                    # self.get_logger().info(f"Position: {tvec}, Orientation: {quat}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
