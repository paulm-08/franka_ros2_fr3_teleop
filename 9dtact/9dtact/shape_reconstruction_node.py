import os
import rclpy
from rclpy.node import Node

import yaml
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .shape_reconstruction import Sensor
from .shape_visualizer import Visualizer
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory


class ShapeReconstructionNode(Node):
    def __init__(self):
        super().__init__('shape_reconstruction_node')

        self.bridge = CvBridge()

        config_path = os.path.join(
            get_package_share_directory('9dtact'),
            'shape_reconstruction',
            'shape_config.yaml'
        )
        with open(config_path, 'r', encoding='utf-8') as f:
            cfg = yaml.load(f, Loader=yaml.FullLoader)

        # Wait for reference image
        ref_image_topic = "/rectify_crop_ref_image"
        self.get_logger().info(f"Waiting for reference image on {ref_image_topic}...")
        ref_msg = self.wait_for_message(ref_image_topic, Image)
        ref = self.bridge.imgmsg_to_cv2(ref_msg, desired_encoding='bgr8')
        self.get_logger().info("Received reference image.")

        self.sensor = Sensor(cfg, ref=ref, open_camera=False)
        self.visualizer = Visualizer(self.sensor.points)
        self.height_map = self.map(cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY))

        image_topic = "/rectify_crop_image"
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.reconstruction_callback,
            10
        )

        self.get_logger().info(f"Subscribed to image topic: {image_topic}")

    def wait_for_message(self, topic, msg_type):
        from rclpy.task import Future
        future = Future()

        def callback(msg):
            future.set_result(msg)

        sub = self.create_subscription(msg_type, topic, callback, 1)
        rclpy.spin_until_future_complete(self, future)
        self.destroy_subscription(sub)
        return future.result()

    def map(self, img_GRAY):
        height_map = self.sensor.raw_image_2_height_map(img_GRAY)
        height_map = self.sensor.expand_image(height_map)
        return height_map

    def reconstruction_callback(self, msg):
        # self.get_logger().info("Received new image for reconstruction.")
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_GRAY = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.height_map = self.map(img_GRAY)

    def run(self):
        self.get_logger().info("Starting visualization loop...")

        while rclpy.ok() and self.visualizer.vis.poll_events():
            # Spin once to allow callbacks
            rclpy.spin_once(self, timeout_sec=0.01)

            # Update visualization
            points, gradients = self.sensor.height_map_2_point_cloud_gradients(self.height_map)
            self.visualizer.update(points, gradients)



def main(args=None):
    rclpy.init(args=args)
    node = ShapeReconstructionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
