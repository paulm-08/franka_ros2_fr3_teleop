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
import numpy as np

package_share = get_package_share_directory('tact9d')
calibration_dir = os.path.join(package_share, 'shape_reconstruction', 'calibration', 'sensor_1', 'camera_calibration')

row_index_path = os.path.join(calibration_dir, 'row_index.npy')
col_index_path = os.path.join(calibration_dir, 'col_index.npy')

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.bridge = CvBridge()

        config_path = os.path.join(
            ament_index_python.packages.get_package_share_directory('tact9d'),
            'shape_reconstruction',
            'shape_config_middle.yaml'
        )
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = yaml.load(f, Loader=yaml.FullLoader)
        self.sensor = Sensor(cfg, package_share_path=package_share)

        self.ref_pub = self.create_publisher(Image, '/rectify_crop_ref_image', 1)
        self.image_pub = self.create_publisher(Image, '/rectify_crop_image', 1)
        self.repr_pub = self.create_publisher(Image, '/deformation_representation', 1)
        self.height_map_pub = self.create_publisher(Image, '/height_map', 1)

        self.publish_reference_image()
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 Hz

    def publish_reference_image(self):
        ref_msg = self.bridge.cv2_to_imgmsg(self.sensor.ref, encoding='bgr8')
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.header.frame_id = 'ref'
        self.ref_pub.publish(ref_msg)

    def timer_callback(self):
        # self.get_logger().info("Timer callback running")
        image = self.sensor.get_rectify_crop_image()
        rep_img, mix_vis = self.sensor.raw_image_2_representation(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        height_map = self.sensor.raw_image_2_height_map(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        height_map = self.sensor.expand_image(height_map)
        height_map_normalized = cv2.normalize(height_map, None, 0, 255, cv2.NORM_MINMAX)
        height_map_uint8 = height_map_normalized.astype(np.uint8)
        height_map_color = cv2.applyColorMap(height_map_uint8, cv2.COLORMAP_JET)

        cv2.imshow('rectify_crop_image', image)
        cv2.imshow('mixed_visualization', mix_vis)
        cv2.imshow('deformation_representation', rep_img)
        cv2.imshow('height_map', height_map_color)
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

        height_map_msg = self.bridge.cv2_to_imgmsg(height_map_color, encoding='bgr8')
        height_map_msg.header.stamp = now
        height_map_msg.header.frame_id = 'height_map'
        self.height_map_pub.publish(height_map_msg)

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

