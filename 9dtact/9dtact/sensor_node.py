import os
import sys
import cv2
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .shape_reconstruction import Sensor
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory

package_share = get_package_share_directory('9dtact')
calibration_dir = os.path.join(package_share, 'shape_reconstruction', 'calibration', 'sensor_1', 'camera_calibration')

row_index_path = os.path.join(calibration_dir, 'row_index.npy')
col_index_path = os.path.join(calibration_dir, 'col_index.npy')

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.bridge = CvBridge()

        config_path = os.path.join(
            ament_index_python.packages.get_package_share_directory('9dtact'),
            'shape_reconstruction',
            'shape_config.yaml'
        )
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = yaml.load(f, Loader=yaml.FullLoader)
        self.sensor = Sensor(cfg, package_share_path=package_share)

        self.ref_pub = self.create_publisher(Image, '/rectify_crop_ref_image', 1)
        self.image_pub = self.create_publisher(Image, '/rectify_crop_image', 1)
        self.repr_pub = self.create_publisher(Image, '/deformation_representation', 1)

        self.publish_reference_image()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 Hz

    def publish_reference_image(self):
        ref_msg = self.bridge.cv2_to_imgmsg(self.sensor.ref, encoding='bgr8')
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.header.frame_id = 'ref'
        self.ref_pub.publish(ref_msg)

    def timer_callback(self):
        self.get_logger().info("Timer callback running")
        image = self.sensor.get_rectify_crop_image()
        rep_img, mix_vis = self.sensor.raw_image_2_representation(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))

        cv2.imshow('rectify_crop_image', image)
        cv2.imshow('mixed_visualization', mix_vis)
        key = cv2.waitKey(1)
        if key == ord('q'):
            rclpy.shutdown()

        now = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        image_msg.header.stamp = now
        image_msg.header.frame_id = 'rectify_crop'
        self.image_pub.publish(image_msg)

        rep_msg = self.bridge.cv2_to_imgmsg(rep_img, encoding='bgr8')
        rep_msg.header.stamp = now
        rep_msg.header.frame_id = 'representation'
        self.repr_pub.publish(rep_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

