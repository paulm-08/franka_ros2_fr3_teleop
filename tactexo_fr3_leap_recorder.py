import argparse   # for handling command-line arguments -s , --store_hand -o
import copy         # deep copy, preventing unintended changes to original data
import numpy as np
import os
from os import path
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

# for getting point cloud
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import time 
import threading

import struct
import pyrealsense2 as rs
import pinocchio as pin
from transforms3d.quaternions import quat2mat

# control leap hand msg 
from leap_hand.srv import LeapPosition, LeapPosVelEff

def save_frame(
    frame_id,
    out_directory,
    color_buffer,
    depth_buffer,
    # color_buffer2,
    # depth_buffer2,
    pc_buffer,
    # pc2_buffer,
    joint_buffer,
    rthumb_raw_buffer,
    rindex_raw_buffer,
    rmiddle_raw_buffer,
    rthumb_deform_buffer,
    rindex_deform_buffer,
    rmiddle_deform_buffer,
):
    # print if there are content in the buffer
    if not color_buffer:
        print("No color buffer")
    # print("Saving frame ", frame_id)
    frame_directory = os.path.join(out_directory, f"frame_{frame_id}")
    os.makedirs(frame_directory, exist_ok=True)

    cv2.imwrite(
        os.path.join(frame_directory, "color_image.jpg"),
        color_buffer[frame_id],
    )
    cv2.imwrite(
        os.path.join(frame_directory, "depth_image.png"), depth_buffer[frame_id]
    )

    # cv2.imwrite(
    #     os.path.join(frame_directory, "color_image2.jpg"),
    #     color_buffer2[frame_id],
    # )
    # cv2.imwrite(
    #     os.path.join(frame_directory, "depth_image2.png"), depth_buffer2[frame_id]
    # )

    # # point cloud
    # o3d.io.write_point_cloud(os.path.join(frame_directory, "pc.ply"), pc_buffer[frame_id])

    # o3d.io.write_point_cloud(os.path.join(frame_directory, "pc2.ply"), pc2_buffer[frame_id])

    # fingertip tactile info
    # cv2.imwrite(
    #     os.path.join(frame_directory, "rthumb_raw_image.jpg"), rthumb_raw_buffer[frame_id]
    # )
    # cv2.imwrite(
    #     os.path.join(frame_directory, "rindex_raw_image.jpg"), rindex_raw_buffer[frame_id]
    # )
    # cv2.imwrite(
    #     os.path.join(frame_directory, "rmiddle_raw_image.jpg"), rmiddle_raw_buffer[frame_id]
    # )
    # cv2.imwrite(
    #     os.path.join(frame_directory, "rthumb_deform_image.jpg"), rthumb_deform_buffer[frame_id]
    # )
    # cv2.imwrite(
    #     os.path.join(frame_directory, "rindex_deform_image.jpg"), rindex_deform_buffer[frame_id]
    # )
    # cv2.imwrite(
    #     os.path.join(frame_directory, "rmiddle_deform_image.jpg"), rmiddle_deform_buffer[frame_id]
    # )


    # joint state
    np.savetxt(
        os.path.join(frame_directory, "right_arm_joint.txt"),
        joint_buffer[frame_id]
    )

    return f"frame {frame_id + 1} saved"


class RobotRecorder(Node):
    def __init__(
        self,
        total_frame,
        out_directory=None
    ):
        super().__init__("TacExo_Real_Record_Data")
        self.save = True
        self.enable_visualization = True
        self.out_directory = out_directory or "recorded_data"
        self.bridge = CvBridge()
        self.sample_rate = 20 # 20 Hz (make it a constant in hyperparameters.py or in input arguments in the furture)
        self.sample_period = 1.0 / self.sample_rate # 1/30 = 0.0333s

        # 101622074637
        self.total_frame = total_frame

        self.joint_state = None

        # Initialize locks for thread-safe data access
        # self.data_lock = threading.Lock()

        self.color = None
        self.depth = None
        self.color2 = None
        self.depth2 = None
        self.pc = None
        # self.pc2 = None
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
        # self.pc2_buffer = []              #  Point cloud 2
        self.joint_buffer = []          #  Exo joint state 
        self.rthumb_raw_buffer = []  # Right thumb raw tactile image
        self.rindex_raw_buffer = []  # Right index raw tactile image
        self.rmiddle_raw_buffer = []  # Right middle raw tactile image
        self.rthumb_deform_buffer = []  # Right thumb deformed tactile image
        self.rindex_deform_buffer = []  # Right index deformed tactile image
        self.rmiddle_deform_buffer = []  # Right middle deformed tactile image

        self.new_msg_received_flag = False

        joint_sub = Subscriber(
            self,
            JointState,
            '/joint_states')
        
        # leap_hand_sub = Subscriber(
        #     self,
        #     Float64MultiArray,
        #     '/leap_hand/position')
        


        thumb_raw_sub = Subscriber(self, Image, '/sensor_soft_06/rectify_crop_image')
        index_raw_sub = Subscriber(self, Image, '/sensor_soft_04/rectify_crop_image')
        middle_raw_sub = Subscriber(self, Image, '/sensor_soft_03/rectify_crop_image')
        thumb_deform_sub = Subscriber(self, Image, '/sensor_soft_06/deformation_representation')
        index_deform_sub = Subscriber(self, Image, '/sensor_soft_04/deformation_representation')
        middle_deform_sub = Subscriber(self, Image, '/sensor_soft_03/deformation_representation')


        self.joint_positions = np.zeros(7)
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

        self.ts = ApproximateTimeSynchronizer(
            [joint_sub],
            queue_size=1,
            slop=self.sample_period - 0.001,  # Adjust slop to match sample period
            allow_headerless=True,
        )

        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info("ApproximateTimeSynchronizer initialized")
        
        # sleep for 2s
        # time_start = time.time()
        # time_count = 0
        # while time_count < 2:
        #     time_count = time.time()-time_start
        #     print(r"wait for 2s:%f",time_count)

        # cam_front_translation = [1.2867936975704506, 0.032497565951025945, 0.5659742126690214]
        cam_front_translation = [1.2367936975704506, 0.032497565951025945, 0.5359742126690214]
       
        cam_front_quaternion = [0.012480230443529135, 0.27804828390806924, -0.026321127948753298, -0.960125301139054]  # [w, x, y, z]
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
        self.camfront2robot = self.camfront2robot @ T

        cam_side_translation = [0.8191031027617821, 0.7523905863952166, 0.5717842949076667]
        cam_side_quaternion = [0.6814160180896522, 0.17193089301379363, 0.20154558868356742, -0.6822692679584189]  # [w, x, y, z]
        # Convert quaternion to rotation matrix
        cam_side_rotation_matrix = quat2mat([cam_side_quaternion[0], cam_side_quaternion[1], cam_side_quaternion[2], cam_side_quaternion[3]])

        
        self.camside2robot = np.eye(4)
        self.camside2robot[:3, :3] = cam_side_rotation_matrix
        self.camside2robot[:3, 3] = cam_side_translation
        self.camside2robot = self.camside2robot @ T
        # self.camfront2robot = np.linalg.inv(self.camside2robot)



        self.leap_position_client = self.create_client(LeapPosition, '/leap_position')

        # Wait for the service to be available
        while not self.leap_position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for /leap_position service...')

        

    def get_current_leap_position(self):
        # Create a request for the LeapPosition service
        req = LeapPosition.Request()
        future = self.leap_position_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return list(future.result().position)
        else:
            self.get_logger().info("Failed to get current position, using zeros")
            return [0.0] * 16



    # for converting plam lower pose to denso end link ( a predefined tf)

    def configure_stream(self):
        print("Configuring Realsense stream...")
        camera_serials = {
            'camera1': '036522072607',
            # 'camera2': '234222300515',
            # Add more cameras as needed
        }

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
        print(self.intrinsics)
        # a = input("Enter")
        # depth_sensor = pipeline_profile.get_device().first_depth_sensor()
        # depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)
        # self.depth_scale = depth_sensor.get_depth_scale()
        align_to = rs.stream.color     # align depth frame to color frame
        self.align = rs.align(align_to)

        # # Configure Camera 2
        # self.pipeline_cam2 = rs.pipeline()
        # config2 = rs.config()
        # config2.enable_device(camera_serials['camera2'])
        # config2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # pipeline_profile2 = self.pipeline_cam2.start(config2)
        # print(f"Started streaming from Camera 2 with serial number {camera_serials['camera2']}")

        # # Get intrinsics for Camera 2
        # self.intrinsics2 = pipeline_profile2.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        # self.intrinsics_cam2 = self.intrinsics2
        # print(f"Intrinsics for Camera 2: {self.intrinsics2}")

        # # Align depth to color frame for Camera 2
        # self.align_cam2 = rs.align(rs.stream.color)

        self.vis = None

        # Initialize Open3D visualization
        print("Initializing Open3D visualization...")
        if self.enable_visualization:
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
            print("Error: No depth or color frame detected")
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
            print("Error: No depth or color frame detected")
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
        print("Synchronized data received")
        

    
    
    def sync_callback(self, joint_state_msg):#, thumb_raw_msg, index_raw_msg, middle_raw_msg, thumb_deform_msg, index_deform_msg, middle_deform_msg):
        # print("Synchronized data received")
        self.new_msg_received_flag  = True
        # Process joint states
        self.joint_state_callback(joint_state_msg)

        # print("leap_hand_positions received")

        # self.front_camera_rgb_callback(front_camera_rgb_msg)
        # self.front_camera_depth_callback(front_camera_depth_msg)
        # # self.side_camera_rgb_callback(side_camera_rgb_msg)
        # # self.side_camera_depth_callback(side_camera_depth_msg)

        

        # self.color = self.bridge.imgmsg_to_cv2(front_camera_rgb_msg, desired_encoding="passthrough")
        # self.depth = self.bridge.imgmsg_to_cv2(front_camera_depth_msg, desired_encoding="passthrough")
        # # self.color2 = self.bridge.imgmsg_to_cv2(side_camera_rgb_msg, desired_encoding="passthrough")
        # # self.depth2 = self.bridge.imgmsg_to_cv2(side_camera_depth_msg, desired_encoding="passthrough")

        # self.thumb_raw_img = self.bridge.imgmsg_to_cv2(thumb_raw_msg, desired_encoding="passthrough")
        # self.index_raw_img = self.bridge.imgmsg_to_cv2(index_raw_msg, desired_encoding="passthrough")
        # self.middle_raw_img = self.bridge.imgmsg_to_cv2(middle_raw_msg, desired_encoding="passthrough")
        # self.thumb_deform_img = self.bridge.imgmsg_to_cv2(thumb_deform_msg, desired_encoding="passthrough")
        # self.index_deform_img = self.bridge.imgmsg_to_cv2(index_deform_msg, desired_encoding="passthrough")
        # self.middle_deform_img = self.bridge.imgmsg_to_cv2(middle_deform_msg, desired_encoding="passthrough")


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
            self.joint_positions = np.array(msg.position[:7])
            self.get_logger().info(f"Joint positions: {self.joint_positions}")
            # Create a mapping from joint name to its position
            joint_dict = {name: pos for name, pos in zip(msg.name, msg.position)}
            # Define the desired joint order based on your URDF
            desired_order = ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"]
            # Extract positions; if a joint is missing, default to 0.0
            joint_positions = [joint_dict.get(joint_name, 0.0) for joint_name in desired_order]
            self.joint_positions = np.array(joint_positions)
            # print names
            self.get_logger().info(f"Joint names: {msg.name}")
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
    

    def process_frame(self):
        self.configure_stream()
        frame_count = 0
        time.sleep(1)
        self.first_frame = True
        o3d_depth_intrinsic = o3d.camera.PinholeCameraIntrinsic(self.intrinsics.width, self.intrinsics.height, self.intrinsics.fx, self.intrinsics.fy, self.intrinsics.ppx, self.intrinsics.ppy)
        # o3d_depth_intrinsic2 = o3d.camera.PinholeCameraIntrinsic(self.intrinsics2.width, self.intrinsics2.height, self.intrinsics2.fx, self.intrinsics2.fy, self.intrinsics2.ppx, self.intrinsics2.ppy)

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
                # rgbd2, self.depth2, self.color2 = self.get_rgbd_frame_from_realsense_cam2(enable_visualization=self.enable_visualization)
                self.pc = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, o3d_depth_intrinsic)
                # self.pc2 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd2, o3d_depth_intrinsic2)
                self.pc.transform(self.camfront2robot)
                # self.pc2.transform(self.camside2robot)


                if self.first_frame:
                    if self.enable_visualization:
                        pcd_vis = self.pc
                        # pcd_vis2 = self.pc2
                        self.vis.add_geometry(pcd_vis)
                        # self.vis.add_geometry(pcd_vis2)

                        robot_base = o3d.geometry.TriangleMesh.create_coordinate_frame(
                            size=0.2
                        )
                        self.vis.add_geometry(robot_base)
                        self.first_frame = False
                else:
                    if self.enable_visualization:
                        pcd_vis.points = self.pc.points
                        pcd_vis.colors = self.pc.colors
                        # pcd_vis2.points = self.pc2.points
                        # pcd_vis2.colors = self.pc2.colors
                        self.vis.update_geometry(pcd_vis)
                        # self.vis.update_geometry(pcd_vis2)
                        self.vis.poll_events()
                        self.vis.update_renderer()
                        
                if self.save:
                    self.color_buffer.append(copy.deepcopy(self.color))
                    self.depth_buffer.append(copy.deepcopy(self.depth))
                    # self.color_buffer2.append(copy.deepcopy(self.color2))
                    # self.depth_buffer2.append(copy.deepcopy(self.depth2))

                    # self.pc_buffer.append(copy.deepcopy(self.pc))
                    # self.pc2_buffer.append(copy.deepcopy(self.pc2))
                    self.joint_buffer.append(copy.deepcopy(self.joint))

                    self.rthumb_raw_buffer.append(copy.deepcopy(self.thumb_raw_img))
                    self.rindex_raw_buffer.append(copy.deepcopy(self.index_raw_img))
                    self.rmiddle_raw_buffer.append(copy.deepcopy(self.middle_raw_img))

                    self.rthumb_deform_buffer.append(copy.deepcopy(self.thumb_deform_img))
                    self.rindex_deform_buffer.append(copy.deepcopy(self.index_deform_img))
                    self.rmiddle_deform_buffer.append(copy.deepcopy(self.middle_deform_img))

                time_end = time.perf_counter()
                time_sleep = max(0, self.sample_period - (time_end - time_start))

                time.sleep(time_sleep)
                

                # print("Frame count: ", frame_count)
                # print("Time sleep: ", time_sleep)
                time_end = time.perf_counter()
                print("Time elapsed: ", time_end - time_start)
                frame_count += 1

        except Exception as e:
            print("An error occurred while processing frames")
            print(e)
        finally:
            print("Frame processing completed")
            print("saving frames...")
            # input("Press Enter to continue...")
            if self.save:
                with concurrent.futures.ThreadPoolExecutor() as executor:
                    futures = [
                        executor.submit(
                            save_frame,
                            frame_id,
                            self.out_directory,
                            self.color_buffer,
                            self.depth_buffer,
                            # self.color_buffer2,
                            # self.depth_buffer2,
                            self.pc_buffer,
                            # self.pc2_buffer,
                            self.joint_buffer,
                            self.rthumb_raw_buffer,
                            self.rindex_raw_buffer,
                            self.rmiddle_raw_buffer,
                            self.rthumb_deform_buffer,
                            self.rindex_deform_buffer,
                            self.rmiddle_deform_buffer
                        )
                        for frame_id in range(frame_count)
                    ]

                    for future in concurrent.futures.as_completed(futures):
                        print(future.result(), f" total frame: {frame_count}")

def main():
    rclpy.init()

    robot_recorder = RobotRecorder(
        total_frame=20,
        out_directory="/home/user/dex-retargeting/recorded_data"
        # out_directory="/home/ruiqiang/workspaces/HK_TacExo/ros2_ws/src/data_recorder/recorded_data/ball_pick_hoh_2025_04_09_01_error_corrections"
    )
    # Initialize the MultiThreadedExecutor and add the node
    executor = MultiThreadedExecutor()
    executor.add_node(robot_recorder)
 
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    print("start process frame")
    robot_recorder.process_frame()

    
    robot_recorder.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
# Set up the argument parser
    # If user chooses to override, remove the existing directory
    main()