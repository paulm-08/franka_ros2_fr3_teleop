#!/usr/bin/env python3

import argparse   # for handling command-line arguments -s , --store_hand -o
import copy         # deep copy, preventing unintended changes to original data
import numpy as np
import os
from os import path
import re

import cv2   # cv2.imwrite

# ROS2 for ExoHand
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge
import concurrent.futures 
import queue
import threading

from sensor_msgs.msg import Image

# for getting point cloud
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import time 
import threading

import struct
import serial
import pyrealsense2 as rs
import pinocchio as pin
from transforms3d.quaternions import quat2mat

# control leap hand msg 
from leap_hand.srv import LeapPosition, LeapPosVelEff

import yaml
from tact9d.shape_reconstruction import Sensor, Visualizer

from multiprocessing import Process, Manager
import threading

from ament_index_python.packages import get_package_share_directory

# TACT_BASE_PATH = '/home/user/franka_ros2_ws/src/tact9d/tact9d/shape_reconstruction/'
TACT_BASE_PATH = get_package_share_directory('tact9d') + '/shape_reconstruction/'

def save_frame(
    frame_id,
    out_directory,
    starting_frame,
    color_buffer,
    depth_buffer,
    color_buffer2,
    depth_buffer2,
    pc_buffer,
    pc2_buffer,
    joint_buffer,
    rthumb_raw_buffer,
    rindex_raw_buffer,
    rmiddle_raw_buffer,
    rthumb_deform_buffer,
    rindex_deform_buffer,
    rmiddle_deform_buffer,
):
    """Save a single frame's data to disk."""
    
    frame_id = frame_id + starting_frame

    # Each buffer is a single-item list containing the frame data
    frame_directory = os.path.join(out_directory, f"frame_{frame_id}")
    os.makedirs(frame_directory, exist_ok=True)

    # Save images
    if color_buffer and color_buffer[0] is not None:
        cv2.imwrite(
            os.path.join(frame_directory, "color_image1.jpg"),
            color_buffer[0],
        )
    if depth_buffer and depth_buffer[0] is not None:
        cv2.imwrite(
            os.path.join(frame_directory, "depth_image1.png"),
            depth_buffer[0]
        )
    if color_buffer2 and color_buffer2[0] is not None:
        cv2.imwrite(
            os.path.join(frame_directory, "color_image2.jpg"),
            color_buffer2[0],
        )
    if depth_buffer2 and depth_buffer2[0] is not None:
        cv2.imwrite(
            os.path.join(frame_directory, "depth_image2.png"),
            depth_buffer2[0]
        )

    # # point cloud (uncomment if needed)
    # if pc_buffer and pc_buffer[0] is not None:
    #     o3d.io.write_point_cloud(os.path.join(frame_directory, "pc.ply"), pc_buffer[0])
    # if pc2_buffer and pc2_buffer[0] is not None:
    #     o3d.io.write_point_cloud(os.path.join(frame_directory, "pc2.ply"), pc2_buffer[0])

    # Fingertip tactile info
    if rthumb_raw_buffer and rthumb_raw_buffer[0] is not None:
        cv2.imwrite(
            os.path.join(frame_directory, "rthumb_raw_image.jpg"), rthumb_raw_buffer[0]
        )
    if rindex_raw_buffer and rindex_raw_buffer[0] is not None:
        cv2.imwrite(
            os.path.join(frame_directory, "rindex_raw_image.jpg"), rindex_raw_buffer[0]
        )
    if rmiddle_raw_buffer and rmiddle_raw_buffer[0] is not None:
        cv2.imwrite(
            os.path.join(frame_directory, "rmiddle_raw_image.jpg"), rmiddle_raw_buffer[0]
        )
    # Uncomment if you want to save deform images
    # if rthumb_deform_buffer and rthumb_deform_buffer[0] is not None:
    #     cv2.imwrite(
    #         os.path.join(frame_directory, "rthumb_deform_image.jpg"), rthumb_deform_buffer[0]
    #     )
    # if rindex_deform_buffer and rindex_deform_buffer[0] is not None:
    #     cv2.imwrite(
    #         os.path.join(frame_directory, "rindex_deform_image.jpg"), rindex_deform_buffer[0]
    #     )
    # if rmiddle_deform_buffer and rmiddle_deform_buffer[0] is not None:
    #     cv2.imwrite(
    #         os.path.join(frame_directory, "rmiddle_deform_image.jpg"), rmiddle_deform_buffer[0]
    #     )


    # joint state
    if joint_buffer and len(joint_buffer) > 0 and joint_buffer[0] is not None:
        np.savetxt(
            os.path.join(frame_directory, "right_arm_joint.txt"),
            joint_buffer[0]
        )

    # print(f"Frame {frame_id + 1} saved.")
    return


class RobotRecorder(Node):
    def __init__(
        self,
        total_frame,
        out_directory=None,
        enable_tactile=True,
        enable_visualization=True,
        enable_haptic=True,
    ):
        super().__init__("TacExo_Real_Record_Data")
        self.save = True
        self.enable_tactile = enable_tactile
        self.enable_visualization = enable_visualization
        self.enable_haptic = enable_haptic
        self.out_directory = out_directory or "recorded_data"
        self.starting_frame = self.get_next_frame_id(out_directory)

        self.bridge = CvBridge()
        self.sample_rate = 20 # 20 Hz (make it a constant in hyperparameters.py or in input arguments in the furture)
        self.sample_period = 1.0 / self.sample_rate # 1/30 = 0.0333s

        # 101622074637
        self.total_frame = total_frame

        if self.save:
            self.frame_queue = queue.Queue(maxsize=100)  # Limit buffer size to 100 frames
            self.saving_thread = threading.Thread(target=self.frame_saver, daemon=True)
            self.saving_thread.start()

        self.joint_state = None

        # Initialize locks for thread-safe data access
        # self.data_lock = threading.Lock()

        self.color = None
        self.depth = None
        self.color2 = None
        self.depth2 = None
        self.pc = None
        self.pc2 = None
        self.joint = None
        self.thumb_raw_img = None
        self.index_raw_img = None
        self.middle_raw_img = None
        self.thumb_deform_img = None
        self.index_deform_img = None
        self.middle_deform_img = None

        self.color_buffer = []            #  D435i color 
        self.depth_buffer = []            #  D435i depth 
        self.color_buffer2 = []           #  D435i 2 color
        self.depth_buffer2 = []           #  D435i 2 depth
        self.pc_buffer = []               #  Point cloud
        self.pc2_buffer = []              #  Point cloud 2
        self.joint_buffer = []          #  Exo joint state 
        self.rthumb_raw_buffer = []  # Right thumb raw tactile image
        self.rindex_raw_buffer = []  # Right index raw tactile image
        self.rmiddle_raw_buffer = []  # Right middle raw tactile image
        self.rthumb_deform_buffer = []  # Right thumb deformed tactile image
        self.rindex_deform_buffer = []  # Right index deformed tactile image
        self.rmiddle_deform_buffer = []  # Right middle deformed tactile image

        self.new_msg_received_flag = False

        # joint_sub = Subscriber(
        #     self,
        #     JointState,
        #     '/joint_states')
        
        # ROS2 subscribers
        joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)
        # joint_sub = Subscriber(self, JointState, '/joint_states')
        
        # leap_hand_sub = Subscriber(
        #     self,
        #     Float64MultiArray,
        #     '/leap_hand/position')
        
        self.joint_positions = np.zeros(6)
        self.leap_hand_positions = np.zeros(16)
        self.front_rgb_image_count = 0
        self.front_depth_image_count = 0
        self.side_rgb_image_count = 0
        self.side_depth_image_count = 0

        # self.ts = ApproximateTimeSynchronizer(
        #     [joint_sub, thumb_raw_sub, index_raw_sub, middle_raw_sub, thumb_deform_sub, index_deform_sub, middle_deform_sub],
        #     queue_size=1,
        #     slop=self.sample_period,  # Adjust slop to match sample period
        #     allow_headerless=True,
        # )

        # self.ts = ApproximateTimeSynchronizer(
        #     [joint_sub, leap_hand_sub],
        #     queue_size=1,
        #     slop=self.sample_period - 0.001,  # Adjust slop to match sample period
        #     allow_headerless=True,
        # )

        # self.ts.registerCallback(self.sync_callback)
        # self.get_logger().info("ApproximateTimeSynchronizer initialized")
        
        # sleep for 2s
        # time_start = time.time()
        # time_count = 0
        # while time_count < 2:
        #     time_count = time.time()-time_start
        #     print(r"wait for 2s:%f",time_count)

        # cam_front_translation = [1.2867936975704506, 0.032497565951025945, 0.5659742126690214]
        cam_front_translation = [1.0322713516713722, 0.006353275596612362, 0.8234645537660135]

        cam_front_quaternion = [0.17954778476421387, -0.6799202218980005, -0.690300391513334, 0.17016596109967214]  # [w, x, y, z]
        # Convert quaternion to rotation matrix
        cam_front_rotation_matrix = quat2mat([cam_front_quaternion[0], cam_front_quaternion[1], cam_front_quaternion[2], cam_front_quaternion[3]])

        self.camfront2robot = np.eye(4)
        self.camfront2robot[:3, :3] = cam_front_rotation_matrix
        self.camfront2robot[:3, 3] = cam_front_translation
        T = np.array([
        [0,  0,  1,  0],  # Maps z -> x
        [-1, 0,  0,  0],  # Maps -x -> y
        [0, -1, 0,  0],  # Maps -y -> z
        [0,  0,  0,  1]   # Homogeneous coordinate unchanged
        ])
        # self.camfront2robot = self.camfront2robot @ T

        cam_side_translation = [0.6254128691870716, -0.6593972515411284, 0.2526695484482261]
        cam_side_quaternion = [0.49543508074736786, -0.8686243685292566, 0.0059154078827518, 0.0008916902936986869]  # [w, x, y, z]
        # Convert quaternion to rotation matrix
        cam_side_rotation_matrix = quat2mat([cam_side_quaternion[0], cam_side_quaternion[1], cam_side_quaternion[2], cam_side_quaternion[3]])

        
        self.camside2robot = np.eye(4)
        self.camside2robot[:3, :3] = cam_side_rotation_matrix
        self.camside2robot[:3, 3] = cam_side_translation
        # self.camside2robot = self.camside2robot @ T
        # self.camfront2robot = np.linalg.inv(self.camside2robot)

        self.leap_position_client = self.create_client(LeapPosition, '/leap_position')

        # Wait for the service to be available
        while not self.leap_position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for /leap_position service...')

        # thumb_raw_sub = Subscriber(self, Image, '/rectify_crop_image')
        # index_raw_sub = Subscriber(self, Image, '/sensor_soft_04/rectify_crop_image')
        # middle_raw_sub = Subscriber(self, Image, '/sensor_soft_03/rectify_crop_image')
        # thumb_deform_sub = Subscriber(self, Image, '/deformation_representation')
        # index_deform_sub = Subscriber(self, Image, '/sensor_soft_04/deformation_representation')
        # middle_deform_sub = Subscriber(self, Image, '/sensor_soft_03/deformation_representation')

        # Tactile sensors
        if self.enable_tactile:
            if enable_visualization:
                self.thumb_pub = self.create_publisher(Image, "/tactile/thumb", 10)
                self.index_pub = self.create_publisher(Image, "/tactile/index", 10)
                self.middle_pub = self.create_publisher(Image, "/tactile/middle", 10)
    
            # tactile configuration loading and init
            # thumb_cfg_path = os.path.join(TACT_BASE_PATH, "shape_config_tacexo_thumb.yaml")
            thumb_cfg_path = os.path.join(TACT_BASE_PATH, "shape_config_thumb.yaml")
            if not os.path.exists(thumb_cfg_path):
                raise FileNotFoundError(f"Configuration file not found: {thumb_cfg_path}")
            thumb_f = open(thumb_cfg_path, 'r+', encoding='utf-8')
            thumb_cfg = yaml.load(thumb_f, Loader=yaml.FullLoader)
            self.thumb_tactile_sensor = Sensor(thumb_cfg)
            # self.thumb_tactile_vis  = Visualizer(self.thumb_tactile_sensor.points)

            # Process index tactile data
            index_cfg_path = os.path.join(TACT_BASE_PATH, "shape_config_index.yaml")
            if not os.path.exists(index_cfg_path):
                raise FileNotFoundError(f"Configuration file not found: {index_cfg_path}")
            index_f = open(index_cfg_path, 'r+', encoding='utf-8')
            index_cfg = yaml.load(index_f, Loader=yaml.FullLoader)
            self.index_tactile_sensor = Sensor(index_cfg)
            # self.index_tactile_vis  = Visualizer(self.index_tactile_sensor.points)

            middle_cfg_path = os.path.join(TACT_BASE_PATH, "shape_config_middle.yaml")
            if not os.path.exists(middle_cfg_path):
                raise FileNotFoundError(f"Configuration file not found: {middle_cfg_path}")
            middle_f = open(middle_cfg_path, 'r+', encoding='utf-8')
            middle_cfg = yaml.load(middle_f, Loader=yaml.FullLoader)
            self.middle_tactile_sensor = Sensor(middle_cfg)
            # self.middle_tactile_vis  = Visualizer(self.middle_tactile_sensor.points)

            self.thumb_points = []
            self.index_points = []
            self.middle_points = []

            self.thumb_height_map = []
            self.index_height_map = []
            self.middle_height_map = []

            self.thumb_heat_map = []
            self.index_heat_map = []
            self.middle_heat_map = []

            self.tac_thumb_lock = threading.Lock()
            self.tac_index_lock = threading.Lock()
            self.tac_middle_lock = threading.Lock()
            self.tac_main_lock = threading.Lock()

            # Start threads for each tactile sensor
            self.start_tac_processing()
            # self.visualize_tactile()

            self.bridge = CvBridge()

    def get_next_frame_id(self, out_directory):
        """
        Find the next frame_id to use by checking existing frame directories.
        """
        if not os.path.exists(out_directory):
            self.get_logger().info("No existing directory found. Starting from frame 0.")
            return 0

        # Find all existing frame directories
        frame_dirs = [
            d for d in os.listdir(out_directory)
            if os.path.isdir(os.path.join(out_directory, d)) and re.match(r"^frame_\d+$", d)
        ]

        if not frame_dirs:
            self.get_logger().info("No existing frame directories found. Starting from frame 0.")
            return 0

        # Extract numeric IDs from directory names and get the max
        existing_ids = [int(d.split("_")[1]) for d in frame_dirs]
        self.get_logger().info(f"Existing directory found: {out_directory}. Starting from frame {max(existing_ids) + 1}.")
        return max(existing_ids) + 1
    
    def haptic_feedback_loop(self):
        
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        except serial.SerialException as e:
            print(f"[HAPTIC] Serial connection failed: {e}")
            return
        
        ser.reset_input_buffer()  # discard any leftover bytes
        time.sleep(0.5)  # Let ESP32 boot

        CROP_TOP = 30
        CROP_BOTTOM = 30
        CROP_LEFT = 30
        CROP_RIGHT = 30

        try:
            while not self.stop_event.is_set():
                try:
                    with self.tac_thumb_lock:
                        thumb_map = np.array(self.thumb_height_map)
                        thumb_map = thumb_map[CROP_TOP:thumb_map.shape[0] - CROP_BOTTOM, CROP_LEFT:thumb_map.shape[1] - CROP_RIGHT].copy()
                    with self.tac_index_lock:
                        index_map = np.array(self.index_height_map)
                        index_map = index_map[CROP_TOP:index_map.shape[0] - CROP_BOTTOM, CROP_LEFT:index_map.shape[1] - CROP_RIGHT].copy()

                    with self.tac_middle_lock:
                        middle_map = np.array(self.middle_height_map)
                        middle_map = middle_map[CROP_TOP:middle_map.shape[0] - CROP_BOTTOM, CROP_LEFT:middle_map.shape[1] - CROP_RIGHT].copy()
                        
                    if len(thumb_map) > 0:
                        pwm_thumb = min(np.max(thumb_map) /3, 0.5)
                    else:
                        pwm_thumb = 0.0
                    if len(index_map) > 0:
                        pwm_index = min(np.max(index_map) /3, 0.5)
                    else:
                        pwm_index = 0.0
                    if len(middle_map) > 0:
                        pwm_middle = min(np.max(middle_map) /3, 0.5)
                    else:
                        pwm_middle = 0.0

                    pwm_vals = [pwm_thumb, pwm_index, pwm_middle, 0.0, 0.0]
                    packet = struct.pack('<5f', *pwm_vals)
                    ser.write(packet)

                    self.get_logger().info(f"[HAPTIC] Sent PWM: {pwm_vals}")
                    time.sleep(0.1)

                except KeyboardInterrupt:
                    self.get_logger().info("[HAPTIC] Keyboard interrupt received.")
                    break
                except Exception as e:
                    self.get_logger().info(f"[HAPTIC] Exception in haptic loop: {e}")
                    continue

        finally:
            pwm_vals = [0.] * 5
            packet = struct.pack('<5f', *pwm_vals)
            ser.write(packet)
            self.get_logger().info("[HAPTIC] Motors stopped.")
            ser.close()


    def process_tactile_data(self, sensor, img_size):
        heat_map = []
        raw_img = []
        points = []
        height_map = []
    
        raw_img = sensor.get_rectify_crop_image()
        img_GRAY = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
        height_map = sensor.raw_image_2_height_map(img_GRAY)

        if self.enable_visualization:
            CROP_TOP = 30
            CROP_BOTTOM = 30
            CROP_LEFT = 30
            CROP_RIGHT = 30

            height_map_cropped = height_map[CROP_TOP:height_map.shape[0] - CROP_BOTTOM,
                                CROP_LEFT:height_map.shape[1] - CROP_RIGHT]

            height_map_cropped = sensor.expand_image(height_map_cropped)

            height_map_normalized = cv2.normalize(height_map_cropped, None, 0, 255, cv2.NORM_MINMAX)
            height_map_uint8 = height_map_normalized.astype(np.uint8)
            height_map_color = cv2.applyColorMap(height_map_uint8, cv2.COLORMAP_JET)

            img_msg = self.bridge.cv2_to_imgmsg(height_map_color, encoding="bgr8")
            if sensor.sensor_id == 1:
                self.thumb_pub.publish(img_msg)
            elif sensor.sensor_id == 2:
                self.index_pub.publish(img_msg)
            elif sensor.sensor_id == 3:
                self.middle_pub.publish(img_msg)

        # heat_map_input = cv2.normalize(height_map, None, 0, 255, cv2.NORM_MINMAX)
        # heat_map_input = np.uint8(heat_map_input)
        # heat_map = cv2.applyColorMap(heat_map_input, cv2.COLORMAP_JET)
        # # Add subtitles to each image
        # cv2.putText(heat_map, "Thumb", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        # # Resize images for display
        # target_size = img_size
        # heat_map = cv2.resize(heat_map, target_size, interpolation=cv2.INTER_LINEAR)
        # points, gradients = sensor.height_map_2_point_cloud_gradients(height_map)

        return raw_img, points, heat_map, height_map

    def process_thumb_tactile(self):
        while not self.stop_event.is_set():  # ✅ Check for stop request
            # start_time = time.time()
            thumb_raw_img, thumb_points, thumb_heat_map, thumb_height_map = self.process_tactile_data(self.thumb_tactile_sensor, (640, 480))
            # print(f"Thumb processing time: {time.time() - start_time:.4f} seconds")

            with self.tac_thumb_lock:
                self.thumb_raw_img = thumb_raw_img
                self.thumb_points = thumb_points
                self.thumb_heat_map = thumb_heat_map
                self.thumb_height_map = thumb_height_map

    def process_index_tactile(self):
        while not self.stop_event.is_set():  # ✅ Check for stop request
            index_raw_img, index_points, index_heat_map, index_height_map = self.process_tactile_data(self.index_tactile_sensor, (640, 480))

            with self.tac_index_lock:
                self.index_raw_img = index_raw_img
                self.index_points = index_points
                self.index_heat_map = index_heat_map
                self.index_height_map = index_height_map
    
    def process_middle_tactile(self):
        while not self.stop_event.is_set():  # ✅ Check for stop request
            middle_raw_img, middle_points, middle_heat_map, middle_height_map = self.process_tactile_data(self.middle_tactile_sensor, (640, 480))

            with self.tac_middle_lock:
                self.middle_raw_img = middle_raw_img
                self.middle_points = middle_points
                self.middle_heat_map = middle_heat_map
                self.middle_height_map = middle_height_map

    def visualize_tactile(self):
        while not self.stop_event.is_set():
            imgs_to_show = []

            with self.tac_thumb_lock:
                if hasattr(self, "thumb_raw_img"):
                    imgs_to_show.append(("Thumb Raw", self.thumb_raw_img))
                if hasattr(self, "thumb_height_map"):
                    imgs_to_show.append(("Thumb Height", self.thumb_height_map))

            with self.tac_index_lock:
                if hasattr(self, "index_raw_img"):
                    imgs_to_show.append(("Index Raw", self.index_raw_img))
                if hasattr(self, "index_height_map"):
                    imgs_to_show.append(("Index Height", self.index_height_map))

            with self.tac_middle_lock:
                if hasattr(self, "middle_raw_img"):
                    imgs_to_show.append(("Middle Raw", self.middle_raw_img))
                if hasattr(self, "middle_height_map"):
                    imgs_to_show.append(("Middle Height", self.middle_height_map))

            # Show images if any available
            for win_name, img in imgs_to_show:
                cv2.imshow(win_name, img)

            # IMPORTANT: this makes imshow work!
            if imgs_to_show:
                # Exit on 'q'
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.stop_event.set()

        # Cleanup when stopping
        cv2.destroyAllWindows()

    def start_tac_processing(self):
        self.stop_event = threading.Event()
        self.threads = []

        def start_thread(name, target):
            thread = threading.Thread(target=target, name=name, daemon=False)
            thread.start()
            self.threads.append(thread)
            self.get_logger().info(f"Started thread: {name}")

        start_thread("thumb_tactile", self.process_thumb_tactile)
        start_thread("index_tactile", self.process_index_tactile)
        start_thread("middle_tactile", self.process_middle_tactile)

        if self.enable_haptic:
            start_thread("haptic_feedback", self.haptic_feedback_loop)

        # if self.enable_visualization:
        #     start_thread("tactile_visualization", self.visualize_tactile)

    def stop_tac_processing(self):
        # Signal all threads to stop
        if hasattr(self, "stop_event"):
            self.stop_event.set()

        # Wait for all threads to exit
        if hasattr(self, "threads"):
            for t in self.threads:
                t.join(timeout=2)
                if t.is_alive():
                    self.get_logger().warn(f"Thread {t.name} did not exit cleanly.")


    def get_current_leap_position(self):
        # Create a request for the LeapPosition service
        # self.get_logger().info("Requesting current Leap position...")
        req = LeapPosition.Request()
        future = self.leap_position_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is not None:
            return list(future.result().position)
        else:
            self.get_logger().info("Failed to get current position, using zeros")
            return [0.0] * 16

    # for converting plam lower pose to denso end link ( a predefined tf)

    def configure_stream(self):
        self.get_logger().info("Configuring Realsense stream...")
        camera_serials = {
            'camera1': '151422254571',
            'camera2': '036522072607',
            # Add more cameras as needed
        }

        # ctx = rs.context()
        # devices = ctx.query_devices()
        # for dev in devices:
        #     dev.hardware_reset()

        # Create a pipeline for d435i
        # Config Camera 1
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(camera_serials['camera1'])
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        pipeline_profile = self.pipeline.start(config)
        self.intrinsics = pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.get_logger().info(f"Intrinsics for Camera 1: {self.intrinsics}")
        # a = input("Enter")
        # depth_sensor = pipeline_profile.get_device().first_depth_sensor()
        # depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
        # self.depth_scale = depth_sensor.get_depth_scale()
        align_to = rs.stream.color     # align depth frame to color frame
        self.align = rs.align(align_to)

        # Configure Camera 2
        self.pipeline_cam2 = rs.pipeline()
        config2 = rs.config()
        config2.enable_device(camera_serials['camera2'])
        config2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline_profile2 = self.pipeline_cam2.start(config2)
        # self.get_logger().info(f"Started streaming from Camera 2 with serial number {camera_serials['camera2']}")

        # Get intrinsics for Camera 2
        self.intrinsics2 = pipeline_profile2.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.get_logger().info(f"Intrinsics for Camera 2: {self.intrinsics2}")

        # Align depth to color frame for Camera 2
        self.align_cam2 = rs.align(rs.stream.color)

        self.vis = None

        # Initialize Open3D visualization
        if self.enable_visualization:
            self.get_logger().info("Initializing Open3D visualization...")
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.vis.get_view_control().change_field_of_view(step=1.0)


    def get_rgbd_frame_from_realsense(self, enable_visualization=False):
        # get d435i data
        # print("Getting RGBD frame from Realsense...")
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            self.get_logger().info("Error: No depth or color frame detected")
            return None, None, None  # Return None values to indicate error

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        # utils.save_image(os.path.join(frame_folder, "color.png"), color_image)
        # cv2.imwrite(os.path.join(frame_folder, "depth.png"), depth_image)



        depth_image_o3d = o3d.geometry.Image(depth_image)
        color_image_o3d = o3d.geometry.Image(color_image)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image_o3d,
            depth_image_o3d,
            depth_trunc=3.0,
            convert_rgb_to_intensity=False,
        )

        return rgbd, depth_image, color_image
    
    def get_rgbd_frame_from_realsense_cam2(self, enable_visualization=False):
        # get d435i data
        # print("Getting RGBD frame from Realsense...")
        frames = self.pipeline_cam2.wait_for_frames()
        aligned_frames = self.align_cam2.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            self.get_logger().info("Error: No depth or color frame detected")
            return None, None, None
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image_o3d = o3d.geometry.Image(depth_image)
        color_image_o3d = o3d.geometry.Image(color_image)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_image_o3d,
            depth_image_o3d,
            depth_trunc=3.0,
            convert_rgb_to_intensity=False,
        )

        return rgbd, depth_image, color_image

    def test_callback(self, joint_state_msg, leap_hand_msg):
        self.new_msg_received_flag  = True
        self.get_logger().info("Synchronized data received")
    
    
    def sync_callback(self, joint_state_msg, thumb_raw_msg, index_raw_msg, middle_raw_msg, thumb_deform_msg, index_deform_msg, middle_deform_msg):
        # print("Synchronized data received")
        self.new_msg_received_flag  = True
        # Process joint states
        self.joint_state_callback(joint_state_msg)

        # print("leap_hand_positions received")

        # self.front_camera_rgb_callback(front_camera_rgb_msg)
        # self.front_camera_depth_callback(front_camera_depth_msg)
        # self.side_camera_rgb_callback(side_camera_rgb_msg)
        # self.side_camera_depth_callback(side_camera_depth_msg)

        

        # self.color = self.bridge.imgmsg_to_cv2(front_camera_rgb_msg, desired_encoding="passthrough")
        # self.depth = self.bridge.imgmsg_to_cv2(front_camera_depth_msg, desired_encoding="passthrough")
        # self.color2 = self.bridge.imgmsg_to_cv2(side_camera_rgb_msg, desired_encoding="passthrough")
        # self.depth2 = self.bridge.imgmsg_to_cv2(side_camera_depth_msg, desired_encoding="passthrough")

        self.thumb_raw_img = self.bridge.imgmsg_to_cv2(thumb_raw_msg, desired_encoding="passthrough")
        self.index_raw_img = self.bridge.imgmsg_to_cv2(index_raw_msg, desired_encoding="passthrough")
        self.middle_raw_img = self.bridge.imgmsg_to_cv2(middle_raw_msg, desired_encoding="passthrough")
        self.thumb_deform_img = self.bridge.imgmsg_to_cv2(thumb_deform_msg, desired_encoding="passthrough")
        self.index_deform_img = self.bridge.imgmsg_to_cv2(index_deform_msg, desired_encoding="passthrough")
        self.middle_deform_img = self.bridge.imgmsg_to_cv2(middle_deform_msg, desired_encoding="passthrough")


    def pointcloud2_to_xyz_rgb(self, msg):
        # Define the structure of the point cloud message with padding
        dtype = np.dtype([
            ('x', np.float32),  # Offset 0
            ('y', np.float32),  # Offset 4
            ('z', np.float32),  # Offset 8
            ('padding', 'V4'),  # Offset 12 (4 bytes padding)
            ('rgb', np.float32) # Offset 16
        ])

        # Convert the buffer to a numpy array
        data = np.frombuffer(msg.data, dtype=dtype)

        # Extract xyz
        xyz = np.vstack([data['x'], data['y'], data['z']]).T

        # Unpack RGB (convert packed float to individual RGB components)
        rgb_int = data['rgb'].view(np.uint32)
        r = (rgb_int >> 16) & 0xFF
        g = (rgb_int >> 8) & 0xFF
        b = rgb_int & 0xFF
        rgb_normalized = np.vstack([r / 255.0, g / 255.0, b / 255.0]).T

        return xyz, rgb_normalized
    
    def print_pointcloud_fields(self, msg):
        for field in msg.fields:
            print(f"Field Name: {field.name}, Offset: {field.offset}, Datatype: {field.datatype}, Count: {field.count}")


    def front_camera_pc_callback(self, front_pc_msg):
        # self.print_pointcloud_fields(front_pc_msg)
        xyz, rgb = self.pointcloud2_to_xyz_rgb(front_pc_msg)
        # Convert to Open3D PointCloud
        self.pc = o3d.geometry.PointCloud()
        self.pc.points = o3d.utility.Vector3dVector(xyz)
        if rgb is not None:
            self.pc.colors = o3d.utility.Vector3dVector(rgb)

    def side_camera_pc_callback(self, side_pc_msg):
        # self.print_pointcloud_fields(side_pc_msg)
        xyz, rgb = self.pointcloud2_to_xyz_rgb(side_pc_msg)
        # Convert to Open3D PointCloud
        self.pc2 = o3d.geometry.PointCloud()
        self.pc2.points = o3d.utility.Vector3dVector(xyz)
        if rgb is not None:
            self.pc2.colors = o3d.utility.Vector3dVector(rgb)

    def joint_state_callback(self, msg):
        """Callback to store joint states."""
        try:
            # self.joint_positions = np.array(msg.position[:6])
            # self.get_logger().info(f"Joint positions: {self.joint_positions}")
            # Create a mapping from joint name to its position
            joint_dict = {name: pos for name, pos in zip(msg.name, msg.position)}
            # Define the desired joint order based on your URDF
            desired_order = ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6","fr3_joint7"]
            # Extract positions; if a joint is missing, default to 0.0
            joint_positions = [joint_dict.get(joint_name) for joint_name in desired_order]
            self.joint_positions = np.array(joint_positions)
            # print names
            # self.get_logger().info(f"Joint names: {msg.name}")
            # print positions
            # self.get_logger().info(f"Joint positions: {self.joint_positions}")
        except Exception as e:
            self.get_logger().error(f"Error processing joint state message: {e}")

    def leap_hand_callback(self, msg):
        try:
            self.leap_hand_positions = np.array(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error processing leap hand positions message: {e}")


    def front_camera_rgb_callback(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


    def front_camera_depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def side_camera_rgb_callback(self, msg):
        self.color2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def side_camera_depth_callback(self, msg):
        self.depth2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def save_joint_buffer_to_file(self, file_path):
        """将 joint_buffer 保存到文件"""
        try:
            np.savetxt(file_path, self.joint_buffer, delimiter=",", fmt="%.6f")
            # self.get_logger().info(f"Joint buffer saved to {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save joint buffer: {e}")
    
    def frame_saver(self):
        while True:
            frame = self.frame_queue.get()
            try:
                frame_id, frame_data = frame
                if frame_id is None:  # sentinel
                    return
                save_frame(frame_id, self.out_directory, self.starting_frame, *frame_data)
                self.get_logger().info(f"Frame {frame_id + 1} saved.")
            except Exception as e:
                # Log but keep the pipeline moving
                self.get_logger().error(f"Error saving frame {frame_id}: {e}")
            finally:
                # Always mark this item done (including the sentinel)
                self.frame_queue.task_done()


    def process_frame(self):
        self.configure_stream()
        frame_count = 0
        time.sleep(0.1)
        self.first_frame = True
        o3d_depth_intrinsic = o3d.camera.PinholeCameraIntrinsic(self.intrinsics.width, self.intrinsics.height, self.intrinsics.fx, self.intrinsics.fy, self.intrinsics.ppx, self.intrinsics.ppy)
        o3d_depth_intrinsic2 = o3d.camera.PinholeCameraIntrinsic(self.intrinsics2.width, self.intrinsics2.height, self.intrinsics2.fx, self.intrinsics2.fy, self.intrinsics2.ppx, self.intrinsics2.ppy)

        try:
            while frame_count < self.total_frame:
                time_start = time.perf_counter()
                # if not self.new_msg_received_flag:
                #     print(self.new_msg_received_flag)
                #     print("No new msg")
                #     raise Exception("No new messages received.")
                # if frame_count == 0:
                #     print("Start recording")
                #     print(self.joint)
                #     print(self.color)
                #     input("Press Enter to continue...")

                self.new_msg_received_flag = False
                self.leap_hand_positions = self.get_current_leap_position()
                self.joint = np.concatenate((self.joint_positions, self.leap_hand_positions))

                rgbd, self.depth, self.color = self.get_rgbd_frame_from_realsense(enable_visualization=self.enable_visualization)
                rgbd2, self.depth2, self.color2 = self.get_rgbd_frame_from_realsense_cam2(enable_visualization=self.enable_visualization)
                self.pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_depth_intrinsic)
                self.pc2 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd2, o3d_depth_intrinsic2)
                self.pc.transform(self.camfront2robot)
                self.pc2.transform(self.camside2robot)


                if self.first_frame:
                    if self.enable_visualization and self.vis is not None:
                        pcd_vis = self.pc
                        pcd_vis2 = self.pc2
                        self.vis.add_geometry(pcd_vis)
                        self.vis.add_geometry(pcd_vis2)

                        robot_base = o3d.geometry.TriangleMesh.create_coordinate_frame(
                            size=0.2
                        )
                        self.vis.add_geometry(robot_base)
                        self.first_frame = False
                else:
                    if self.enable_visualization and self.vis is not None:
                        pcd_vis.points = self.pc.points
                        pcd_vis.colors = self.pc.colors
                        pcd_vis2.points = self.pc2.points
                        pcd_vis2.colors = self.pc2.colors
                        self.vis.update_geometry(pcd_vis)
                        self.vis.update_geometry(pcd_vis2)
                        self.vis.poll_events()
                        self.vis.update_renderer()
                        
                if self.save:
                    frame_data = (
                        [copy.deepcopy(self.color)],
                        [copy.deepcopy(self.depth)],
                        [copy.deepcopy(self.color2)],
                        [copy.deepcopy(self.depth2)],
                        [copy.deepcopy(self.pc)],
                        [copy.deepcopy(self.pc2)],
                        [copy.deepcopy(self.joint)],
                        [copy.deepcopy(self.thumb_raw_img)],
                        [copy.deepcopy(self.index_raw_img)],
                        [copy.deepcopy(self.middle_raw_img)],
                        [copy.deepcopy(self.thumb_deform_img)],
                        [copy.deepcopy(self.index_deform_img)],
                        [copy.deepcopy(self.middle_deform_img)],
                    )
                    self.frame_queue.put((frame_count, frame_data))
                    # print(f"Enqueued frame {frame_count + 1} for saving.")

                time_end = time.perf_counter()
                time_sleep = max(0, self.sample_period - (time_end - time_start))

                time.sleep(time_sleep)
                

                # print("Frame count: ", frame_count)
                # print("Time sleep: ", time_sleep)
                time_end = time.perf_counter()
                # print("Time elapsed: ", time_end - time_start)
                frame_count += 1

        except Exception as e:
            self.get_logger().info("An error occurred while processing frames")
            self.get_logger().info(e)
            
        finally:
            if self.enable_visualization and self.vis is not None:
                self.vis.destroy_window()

            if self.save:
                if self.enable_tactile:
                    if not os.path.exists(os.path.join(self.out_directory, "tactile_ref")):
                        os.makedirs(os.path.join(self.out_directory, "tactile_ref"))
                    cv2.imwrite(os.path.join(self.out_directory, "tactile_ref/rthumb_raw_reference.jpg"), self.thumb_tactile_sensor.ref)
                    cv2.imwrite(os.path.join(self.out_directory, "tactile_ref/rindex_raw_reference.jpg"), self.index_tactile_sensor.ref)
                    cv2.imwrite(os.path.join(self.out_directory, "tactile_ref/rmiddle_raw_reference.jpg"), self.middle_tactile_sensor.ref)
                    self.get_logger().info("Tactile reference images saved.")
                # Send sentinel to stop the saver thread
                self.frame_queue.put((None, None))
                self.saving_thread.join()

def main():
    parser = argparse.ArgumentParser(description="Record FR3 Leap data")
    parser.add_argument(
        "-o", "--out_directory",
        type=str,
        default="/home/user/recorded_data/test",
        help="Output directory to save recorded data"
    )
    parser.add_argument(
        "-n", "--total_frame",
        type=int,
        default=10000,
        help="Number of frames to record"
    )
    args = parser.parse_args()
    rclpy.init()

    robot_recorder = RobotRecorder(
        total_frame=args.total_frame,
        out_directory=args.out_directory,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(robot_recorder)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    try:
        print("Start process frame")
        robot_recorder.process_frame()
        print("Process frame finished")
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Shutting down...")
    finally:
        robot_recorder.stop_tac_processing()
        executor.shutdown()
        robot_recorder.destroy_node()
        executor_thread.join(timeout=1.0)

        if rclpy.ok():
            rclpy.shutdown()        # Optionally join the thread if not daemon
    return 0

if __name__ == "__main__":
# Set up the argument parser
    # If user chooses to override, remove the existing directory
    main()