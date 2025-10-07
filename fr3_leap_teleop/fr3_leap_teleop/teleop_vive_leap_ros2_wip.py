#!/usr/bin/env python3

# import sys
# sys.path.append("/home/user/dex-retargeting/dex_retargeting")  # Adjust to actual repo path
# sys.path.append("/home/user/dex-retargeting/example/vector_retargeting")

# import os
# from launch.actions import SetEnvironmentVariable

# # --- Set PYTHONPATH for dex-retargeting ---
# SetEnvironmentVariable(
#     name='PYTHONPATH',
#     value='/home/user/dex-retargeting:' + os.environ.get('PYTHONPATH', '')
# )

import multiprocessing
import asyncio
import json
import open3d as o3d
import numpy as np
import cv2
import time

import tyro
from loguru import logger

from pathlib import Path
from queue import Empty

from dex_retargeting.constants import RobotName, RetargetingType, HandType, get_default_config_path
from dex_retargeting.retargeting_config import RetargetingConfig

from dex_retargeting.single_hand_detector import SingleHandDetector

from dex_retargeting.leap_hand_utils.dynamixel_client import *
import dex_retargeting.leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import LeapPosition, LeapVelocity, LeapEffort, LeapPosVelEff

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

from geometry_msgs.msg import PoseStamped, TwistStamped

from scipy.spatial.transform import Rotation as R

from std_srvs.srv import Trigger

import argparse

import pyrealsense2 as rs

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import JointState

# --- Configuration and Initialization ---
target_rate = 30  # Target loop rate in Hz

teleop_mode = "side_to_side"  # "side_to_side" or "mirror"

hand = True  # Whether to use a hand
ee_id = "leap_hand"  # End effector ID
hand_control = True  # Whether to control the hand
arm_control = True  # Whether to control the arm

camera_type = "generic"  # "realsense" or "generic"

# Define a color map for tracker IDs
tracker_color_map = {
    "left_elbow": [1.0, 0.0, 0.0],  # Red
    "right_elbow": [0.0, 1.0, 0.0],  # Green
    "chest": [0.0, 0.0, 1.0],  # Blue
}

def get_color_for_tracker(tracker_id):
    """
    Get color for a tracker ID.
    """
    return tracker_color_map.get(tracker_id, [0.5, 0.5, 0.5])  # Default to gray if ID is unknown

def update_coordinate_frames(poses, vis):
    """
    Update the coordinate frames in the Open3D visualizer based on the latest pose data.
    """
    start_time = time.perf_counter()

    # Clear all existing geometries
    vis.clear_geometries()

    # Add new geometries
    coordinate_frames = []
    for pose in poses:
        position = pose["position"]
        orientation = pose["orientation"]
        tracker_id = pose.get("tracker_id", "unknown_tracker")

        pos = np.array([position["x"], position["y"], position["z"]])
        q = [orientation["w"], orientation["x"], orientation["y"], orientation["z"]]

        # Get color for the tracker ID
        color = get_color_for_tracker(tracker_id)

        # Create and position the coordinate frame
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        frame.translate(pos)
        frame.rotate(o3d.geometry.get_rotation_matrix_from_quaternion(q), center=pos)

        # Apply color to the axes
        for triangle in frame.triangles:
            frame.paint_uniform_color(color)

        vis.add_geometry(frame)
        coordinate_frames.append(frame)

    # Force Open3D to refresh the scene
    vis.poll_events()
    vis.update_renderer()

    end_time = time.perf_counter()
    # print(f"[Timing] update_coordinate_frames: {end_time - start_time:.6f} seconds")

# --- VIVE UDP Receiver ---
class UDPReceiverProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.poses = []

    def datagram_received(self, data, addr):
        try:
            pose_data = json.loads(data.decode('utf-8'))
            self.poses = pose_data.get("poses", [])
        except Exception as e:
            print(f"Error processing data: {e}")

async def render_loop(protocol, visualize):
    """
    Periodically update the Open3D visualization with the latest pose data.
    """
    while True:
        start_time = time.perf_counter()

        # Update visualization only if there are poses
        if protocol.poses and visualize:
            update_coordinate_frames(protocol.poses)

        end_time = time.perf_counter()
        # print(f"[Timing] render_loop: {end_time - start_time:.6f} seconds")
        # await asyncio.sleep(0.01)  # 30 FPS            

class LeapNode(Node):
    def __init__(self):
        super().__init__('leaphand_node')
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 550
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))

        self.create_service(LeapPosition, '/leap_position', self.pos_srv)
        self.get_logger().info("Leap service /leap_position created")
        # self.create_service(LeapVelocity, 'leap_velocity', self.vel_srv)
        # self.create_service(LeapEffort, 'leap_effort', self.eff_srv)
        # self.create_service(LeapPosVelEff, 'leap_pos_vel_eff', self.pos_vel_eff_srv)
        # self.create_service(LeapPosVelEff, 'leap_pos_vel', self.pos_vel_srv)
           
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM13', 4000000)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less


        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        # #Lower current limit for tip motors
        # self.dxl_client.sync_write([3,7,11,15], np.ones(4) * (self.curr_lim * 0.02), 102, 2)

        # pinch_motors = [1, 2, 3, 13, 14, 15] # Motors for pinch fingers
        # self.dxl_client.sync_write(pinch_motors, np.ones(len(pinch_motors)) * 1000, 84, 2)  # Higher P gain
        # self.dxl_client.sync_write(pinch_motors, np.ones(len(pinch_motors)) * 300, 80, 2) # Higher D gain
        # self.dxl_client.sync_write(pinch_motors, np.ones(len(pinch_motors)) * 1300, 102, 2)  # Higher current limit

        if hand_control:
            self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        else:
            logger.info("[WARNING] Hand control is disabled.")

        self.publisher_ = self.create_publisher(JointState, '/leap_hand_joint_cmd', 10)
        self.publisher2_ = self.create_publisher(JointState, '/leap_hand_joint_state', 10)
        self._joint_names = [f"leap_joint_{i}" for i in range(16)]

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position
    def read_pos(self):
        return self.dxl_client.read_pos()
    # Service that reads and returns the pos of the robot in regular LEAP Embodiment scaling.
    def pos_srv(self, request, response):
        # self.get_logger().info("Received /leap_position request")
        response.position = self.dxl_client.read_pos().tolist()
        return response
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
    def cleanup(self):
        # Send relax command to all motors
        try:
            self.dxl_client.set_torque_enabled(self.motors, False)
            print("[LEAP] Motors relaxed.")
        except Exception as e:
            print(f"[LEAP] Failed to relax motors: {e}")

def start_retargeting(queue: multiprocessing.Queue, robot_dir: str, config_path: str, shutdown_event):

    def unit(v):
        n = np.linalg.norm(v)
        if n < 1e-12:
            return np.zeros_like(v)
        return v / n

    def fingertip_frame_from_keypoints(tip, dip=None, pip=None, mcp=None, palm_normal=None):
        """
        Estimate fingertip frame (normal, side, axis) from available keypoints.
        Keypoints: tip, dip, pip, mcp are 3D numpy arrays or None if unavailable.

        Strategy:
        - Axis is always defined as tip->dip if dip is given, else tip->pip, else fallback to tip->mcp.
        - Secondary vector is chosen from (dip->mcp, dip->pip, pip->tip) in that priority.
        - Normal is cross(axis, secondary).
        - If palm_normal is given, flip sign to be consistent.
        - Side is cross(normal, axis).
        """

        # --- Step 1: axis (tip direction) ---
        if dip is not None:
            axis = unit(tip - dip)
        elif pip is not None:
            axis = unit(tip - pip)
        elif mcp is not None:
            axis = unit(tip - mcp)
        else:
            raise ValueError("At least one of dip, pip, or mcp must be provided to define axis.")

        # --- Step 2: choose secondary vector ---
        candidates = []
        if dip is not None and mcp is not None:
            candidates.append(dip - mcp)
        if dip is not None and pip is not None:
            candidates.append(dip - pip)
        if pip is not None:
            candidates.append(tip - pip)  # fallback: along axis itself

        # Pick candidate giving largest cross product magnitude
        best_n = None
        best_mag = -1.0
        for b in candidates:
            n = np.cross(axis, b)
            mag = np.linalg.norm(n)
            if mag > best_mag:
                best_mag = mag
                best_n = n

        if best_n is None or best_mag < 1e-6:
            # Fallback: if everything is collinear, use palm_normal if available
            if palm_normal is None:
                raise ValueError("Unable to estimate normal: insufficient or collinear keypoints.")
            normal = unit(palm_normal)
        else:
            normal = unit(best_n)

        # --- Step 3: flip with palm_normal if available ---
        if palm_normal is not None:
            if np.dot(normal, palm_normal) < 0:
                normal = -normal

        # --- Step 4: compute side vector ---
        side = unit(np.cross(normal, axis))

        return normal, side, axis

    rclpy.init()
    leaphand = LeapNode()

    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    logger.info(f"Start retargeting with config {config_path}")
    retargeting = RetargetingConfig.load_from_file(config_path).build()

    logger.info(f"Retargeting config loaded: {retargeting}")
    logger.info(f"Retargeting type: {retargeting.optimizer.retargeting_type}")
    logger.info(f"Num fingers: {retargeting.optimizer.num_fingers}")

    # hand_type = "Right" if "right" in config_path.lower() else "Left"
    # detector = SingleHandDetector(hand_type=hand_type, selfie=False)

    # Different robot loader may have different orders for joints
    # sapien_joint_names = [joint.get_name() for joint in robot.get_active_joints()]
    # retargeting_joint_names = retargeting.joint_names
    # retargeting_to_sapien = np.array([retargeting_joint_names.index(name) for name in sapien_joint_names]).astype(int)

    # Start executor in a background thread so services/callbacks run
    executor = MultiThreadedExecutor()
    executor.add_node(leaphand)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # time.sleep(1)
    # logger.info(f"Executor thread alive? {executor_thread.is_alive()}")

    try:
        loop_counter = 0
        print_interval = 100  # Print every 100 loops
        last_print_time = time.time()
        while not shutdown_event.is_set():
            try:
                start = time.time()
                joint_pos = queue.get(timeout=5)
                queue_time = time.time() - start

            except Empty:
                # logger.error(f"Fail to fetch image from camera in 5 secs. Please check your web camera device.")
                print("[ERROR] No hand keypoints received in 5 seconds, retrying...")
                continue
            
            start = time.time()
            # _, joint_pos, _, _ = detector.detect(rgb)
            detect_time = time.time() - start

            if joint_pos is None:
                # logger.warning(f"{hand_type} hand is not detected.")
                pass
            else:
                retargeting_type = retargeting.optimizer.retargeting_type
                indices = retargeting.optimizer.target_link_human_indices
                if retargeting_type == "POSITION":
                    indices = indices
                    ref_value = joint_pos[indices, :]
                else:
                    origin_indices = indices[0, :]
                    task_indices = indices[1, :]
                    ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]
                    orientation_targets = None
                    
                    if retargeting_type == "DEXPILOTADAPTED":
                        # --- Extract human fingertip orientations ---
                        # indices for hand keypoints
                        index_mcp_idx, index_pip_idx, index_dip_idx, index_tip_idx = 5, 6, 7, 8  # index MCP, PIP, DIP, tip
                        thumb_mcp_idx, thumb_ip_idx, thumb_tip_idx = 1, 2, 3  # thumb MCP, IP, tip
                        # assume joint_pos is Nx3 array of human keypoints
                        tip_thumb = joint_pos[thumb_tip_idx]
                        ip_thumb = joint_pos[thumb_ip_idx]
                        mcp_thumb = joint_pos[thumb_mcp_idx]
                        tip_index = joint_pos[index_tip_idx]
                        dip_index = joint_pos[index_dip_idx]
                        pip_index = joint_pos[index_pip_idx]
                        mcp_index = joint_pos[index_mcp_idx]
                        # palm_normal = compute_palm_normal(...)  # from wrist/palm keypoints
                        # palm_normal = unit(np.cross(mcp_index - mcp_thumb, pip_index - mcp_thumb))
                        palm_normal = None  # not used for now

                        n_t, s_t, a_t = fingertip_frame_from_keypoints(tip=tip_thumb, dip=ip_thumb, mcp=mcp_thumb, palm_normal=palm_normal)
                        n_i, s_i, a_i = fingertip_frame_from_keypoints(tip=tip_index, dip=dip_index, pip=pip_index, mcp=mcp_index, palm_normal=palm_normal)

                        alpha_h = np.arccos(np.clip(np.dot(n_t, n_i), -1.0, 1.0))
                        # compute beta_h via projection/atan2 as described above
                        beta_h = compute_twist_angle(n_t, s_t, n_i, s_i)

                        orientation_targets = {
                        'normal': [n_t.tolist(), n_i.tolist()],
                        'side':   [s_t.tolist(), s_i.tolist()],
                        'rel_angles': [float(alpha_h), float(beta_h)],
                        # 'confidence': float(confidence)
                        }
                        qpos = retargeting.retarget(ref_value, orientation_targets)

                start = time.time()
                qpos = retargeting.retarget(ref_value, orientation_targets=None) # ,np.array([-0.3,-1.0])) # fixed q_pos
                retarget_time = time.time() - start

                # print(f"[DEBUG] detect={detect_time*1000:.1f} ms, retarget={retarget_time*1000:.1f} ms, queue={queue_time*1000:.1f} ms, total={(retarget_time+detect_time+queue_time)*1000:.1f} ms")

                # logger.info(f"Link names: {retargeting.optimizer.link_names}\n")
                # logger.info(f"Computed link names: {retargeting.optimizer.computed_link_names}\n")
                # logger.info(f"Origin link names: {retargeting.optimizer.origin_link_names}\n")
                # logger.info(f"Task link names: {retargeting.optimizer.task_link_names}\n")
                # logger.info(f"Projected: {retargeting.optimizer.projected}\n")
                # logger.info(f"Project index origin: {retargeting.optimizer.s2_project_index_origin}\n")
                # logger.info(f"Project index task: {retargeting.optimizer.s2_project_index_task}\n")
                # logger.info(f"Projected dist: {retargeting.optimizer.projected_dist}\n")
                # logger.info(f"Target vector: {retargeting.optimizer.target_vec_dist}\n")
                # logger.info(f"qpos: {qpos}\n")
                # print("qpos: " + ", ".join(f"{pos:.4f}" for pos in qpos))

                # logger.info(f"Body position: {retargeting.optimizer.body_pos}")
                # if retargeting.optimizer.forward_parallel_loss is not None:
                #     logger.info(f"forward parallelism loss: {retargeting.optimizer.forward_parallel_loss}")
                # if retargeting.optimizer.side_parallel_loss is not None:
                #     logger.info(f"side parallelism loss: {retargeting.optimizer.side_parallel_loss}")
                # logger.info(f"Huber distance loss: {retargeting.optimizer.huber_distance}")

                qpos_cmd = np.zeros(16)

                if teleop_mode == "side_to_side":
                                    
                    #Index
                    qpos_cmd[0] = qpos[1] # rotation
                    qpos_cmd[1] = qpos[0] # base
                    qpos_cmd[2] = qpos[2] # middle
                    qpos_cmd[3] = qpos[3] # tip

                    # Middle finger
                    qpos_cmd[4] = qpos[9] # rotation
                    qpos_cmd[5] = qpos[8] # base
                    qpos_cmd[6] = qpos[10] # middle
                    qpos_cmd[7] = qpos[11] # tip

                    # Pinky 
                    qpos_cmd[8] = qpos[13] # rotation
                    qpos_cmd[9] = qpos[12] # base
                    qpos_cmd[10] = qpos[14] # middle
                    qpos_cmd[11] = qpos[15] # tip

                    # Thumb
                    qpos_cmd[12] = qpos[4] # base 
                    qpos_cmd[13] = qpos[5] # rotation
                    qpos_cmd[14] = qpos[6] # middle
                    qpos_cmd[15] = qpos[7] # tip

                elif teleop_mode == "mirror":
                    qpos_cmd[0] = -qpos[1]
                    qpos_cmd[1] = qpos[0]
                    qpos_cmd[2] = qpos[2]
                    qpos_cmd[3] = qpos[3]

                    qpos_cmd[4] = -qpos[9] # thumb - middle
                    qpos_cmd[5] = qpos[8]
                    qpos_cmd[6] = qpos[10]
                    qpos_cmd[7] = qpos[11]

                    qpos_cmd[8] = -qpos[13] # none
                    qpos_cmd[9] = qpos[12]
                    qpos_cmd[10] = qpos[14]
                    qpos_cmd[11] = qpos[15]

                    qpos_cmd[12] = -qpos[4] # thumb - middle 
                    qpos_cmd[13] = qpos[5]
                    qpos_cmd[14] = qpos[6]
                    qpos_cmd[15] = qpos[7]
                
                else:
                    print(f"[ERROR] Unknown teleop mode: {teleop_mode}. Use 'mirror' or 'side_to_side'.")
                    continue

                # qpos_cmd[8] = qpos[8]        

                # --- Print actual loop rate ---
                if loop_counter % print_interval == 0:
                    now = time.time()
                    elapsed = now - last_print_time
                    rate = print_interval / elapsed if elapsed > 0 else 0
                    print(f"[INFO] Retargeting publishing rate: {rate:.2f} Hz over last {print_interval} loops", flush=True)
                    last_print_time = now
                    
                loop_counter += 1   

                if hand_control:
                    leaphand.set_allegro(qpos_cmd)

                # msg = JointState()
                # msg.header.stamp = leaphand.get_clock().now().to_msg()
                # msg.name = leaphand._joint_names
                # msg.position = qpos_cmd.tolist()
                # leaphand.publisher_.publish(msg)

                # msg2 = JointState()
                # msg2.header.stamp = leaphand.get_clock().now().to_msg()
                # msg2.name = leaphand._joint_names
                # msg2.position = leaphand.read_pos().tolist()
                # leaphand.publisher2_.publish(msg2)
            # logger.info(f"Executor thread alive? {executor_thread.is_alive()}")
    
    except Exception:
        logger.exception("Exception in child retargeting loop")

    finally:
        # orderly shutdown in child:
        try:
            executor.shutdown(timeout_sec=1.0)
        except Exception:
            logger.exception("Executor shutdown failed in child")
        try:
            executor_thread.join(timeout=1.0)
        except Exception:
            pass

        # try to disable motors cleanly
        try:
            if hasattr(leaphand, "dxl_client") and leaphand.dxl_client is not None:
                try:
                    leaphand.dxl_client.set_torque_enabled(leaphand.motors, False)
                except Exception:
                    logger.exception("Failed to disable torque in child")
        except Exception:
            pass

        try:
            leaphand.cleanup()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        logger.info("start_retargeting: child exiting cleanly")

# --- Camera Producer (runs in a process) ---
def produce_camera_frame(queue: multiprocessing.Queue, camera_path=None, visualize=True):
    """
    Captures frames from a generic USB camera, detects hand keypoints, optionally draws skeleton,
    and pushes RGB frames to a multiprocessing queue.

    Args:
        queue (multiprocessing.Queue): Queue to send RGB frames.
        camera_path (str, optional): Path to camera device (e.g., '/dev/video0').
        visualize (bool): Whether to display frames with drawn skeleton.
    """
    # Initialize hand detector
    detector = SingleHandDetector(hand_type="Right", selfie=False)
    print("[INFO] SingleHandDetector initialized.")

    # calib_file = Path(__file__).parent / "fisheye_calib.npz"
    # calib_file = Path('/home/user/franka_ros2_ws/src/fr3_leap_teleop/fr3_leap_teleop/fisheye_calib.npz')
    pkg_share_dir = get_package_share_directory('fr3_leap_teleop')
    calib_file = Path(pkg_share_dir) / "configs" / "fisheye_calib.npz"

    if Path(calib_file).exists():
        data = np.load(calib_file)
        K, D = data["K"], data["D"]
        map1, map2 = None, None  # 延後初始化
        print("Loaded fisheye calibration.")
    else:
        K, D = None, None
        map1, map2 = None, None
        print("No fisheye calibration file found. Using raw frames.")

    # Open the camera (default: /dev/video0)
    cap = cv2.VideoCapture(camera_path or '/dev/video0', cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[ERROR] Failed to open camera: {camera_path or '/dev/video0'}")
        return
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv2.CAP_PROP_FPS, 25)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
    # print(cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))
    # cap.set(cv2.CAP_PROP_GAIN, 200) 
    # print(cap.get(cv2.CAP_PROP_GAIN))
    cap.set(cv2.CAP_PROP_EXPOSURE, 200)  
    # print(cap.get(cv2.CAP_PROP_EXPOSURE))

    print(f"[INFO] Camera opened: {camera_path or '/dev/video0'}")

    print("FPS:", cap.get(cv2.CAP_PROP_FPS))


    try:
        while True:
            start_time = time.time()
            success, frame = cap.read()
            read_time = time.time() - start_time
            # print(f"[DEBUG] Frame read time: {read_time*1000:.1f} ms")

            if not success:
                print("[WARNING] Failed to read frame, retrying...")
                continue

            # Convert BGR → RGB
            if K is not None and D is not None:
                h, w = frame.shape[:2]
                if map1 is None or map2 is None:
                    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                        K, D, (w, h), np.eye(3), balance=0.0
                    )
                    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                        K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2
                    )
                frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Detect hand keypoints
            _, joint_pos, keypoints, _ = detector.detect(rgb_image)

            # Push keypoints to queue for LEAP hand control
            queue.put(joint_pos)

            if visualize:
                # Draw skeleton if keypoints are detected
                if keypoints is not None:
                    annotated_image = SingleHandDetector.draw_skeleton_on_image(
                        frame, keypoints, style="white"
                    )
                else:
                    annotated_image = frame

                # Show the annotated frame
                cv2.imshow("USB Camera", annotated_image)

                # Exit on 'q'
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except Exception as e:
        print(f"[ERROR] Exception in camera loop: {e}")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Camera stream stopped.")

def produce_realsense_frame(queue: multiprocessing.Queue, serial_number=None, visualize=True):

    # Initialize hand detector
    detector = SingleHandDetector(hand_type="Right", selfie=False)
    print("[INFO] SingleHandDetector initialized.")

    pipeline = rs.pipeline()
    print("[INFO] RealSense pipeline created.")
    config = rs.config()
    if serial_number:
        print(f"[INFO] Enabling RealSense device with serial: {serial_number}")
        config.enable_device(serial_number)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  

    try:
        pipeline.start(config)
        print("[INFO] RealSense D405 camera started.")
    except Exception as e:
        print(f"[ERROR] Failed to start RealSense pipeline: {e}")
        return

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert frame to numpy array and RGB format
            color_image = np.asanyarray(color_frame.get_data())
            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            # Detect hand keypoints
            _,joint_pos,keypoints,_ = detector.detect(rgb_image)
            queue.put(rgb_image)

            if visualize:
                if keypoints is not None:
                    # Draw the skeleton on the original BGR image
                    annotated_image = SingleHandDetector.draw_skeleton_on_image(color_image, keypoints, style="white")
                else:
                    annotated_image = color_image

                cv2.imshow("RealSense", annotated_image)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] RealSense pipeline stopped.")

class ViveToROS2Publisher(Node):
    def __init__(self):
        super().__init__('vive_to_ros2_publisher')
        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.relative_publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.publisher_ = self.create_publisher(PoseStamped, '/vive_pose', 10)
        self.alpha = 0.2  # Smoothing factor for EMA
        self._filter_state = {}  # Store filtered values for each key

        self.prev_tracker_pos = None    # Previous tracker position for velocity calculation
        self.prev_tracker_ori = None    # Previous tracker orientation for velocity calculation
        self.prev_time = None   # Previous time for velocity calculation

        self.robot_current_pose = None  # Current pose of the robot, updated from the robot state broadcaster
        self.robot_initial_pose = None  # Initial pose of the robot, updated from the robot state broadcaster
        self.tracker_initial_pos = None # Initial position of the tracker, set when resetting origin
        self.tracker_initial_ori = None # Initial orientation of the tracker, set when resetting origin
        self.current_tracker_pos = None # Current position of the tracker, updated from VIVE data
        self.current_tracker_ori = None # Current orientation of the tracker, updated from VIVE data
        self.tracking_enabled = False    # Flag to enable/disable tracking
        self.robot_pose_subscription = self.create_subscription(    
            PoseStamped, '/franka_robot_state_broadcaster/current_pose', self.robot_pose_callback, 1)   # Subscribe to robot pose updates
        self.srv = self.create_service(Trigger, '/reset_tracking_origin', self.reset_origin_callback)    # Service to reset tracking origin
        self.alpha = 0.2  # Smoothing factor for velocity

    def robot_pose_callback(self, msg):
        self.robot_current_pose = msg  # Always update, so we have the latest

    def reset_origin_callback(self, request, response):
        if self.current_tracker_pos is not None and self.current_tracker_ori is not None and self.robot_current_pose is not None:
            self.tracker_initial_pos = self.current_tracker_pos
            self.tracker_initial_ori = self.current_tracker_ori
            self.robot_initial_pose = self.robot_current_pose  # Use the latest robot pose as the initial pose
            # robot_initial_pose is already up-to-date from callback
            self.tracking_enabled = True
            response.success = True
            response.message = "Tracking origin reset and tracking enabled."
        else:
            print("current_tracker_pos:", self.current_tracker_pos)
            print("current_tracker_ori:", self.current_tracker_ori)
            print("robot_current_pose:", self.robot_current_pose)
            response.success = False
            response.message = "Tracker or robot pose not available."
        return response
    
    def publish_relative_pose(self, tracker_pos, tracker_ori):
        # Wait until both initial poses are set
        if self.robot_initial_pose is None or self.tracker_initial_pos is None:
            return

        # --- Filtering ---
        tracker_pos = self.filter_value(tracker_pos, key="rel_pos")
        tracker_ori = self.filter_value(tracker_ori, key="rel_ori", is_orientation=True)

        # Compute tracker delta (current - initial)
        delta_pos = tracker_pos - self.tracker_initial_pos
        delta_ori = R.from_quat(tracker_ori) * R.from_quat(self.tracker_initial_ori).inv()

        # Apply delta to robot initial pose
        robot_init_pos = np.array([
            self.robot_initial_pose.pose.position.x,
            self.robot_initial_pose.pose.position.y,
            self.robot_initial_pose.pose.position.z
        ])
        robot_init_ori = [
            self.robot_initial_pose.pose.orientation.x,
            self.robot_initial_pose.pose.orientation.y,
            self.robot_initial_pose.pose.orientation.z,
            self.robot_initial_pose.pose.orientation.w
        ]

        new_pos = robot_init_pos + delta_pos
        new_ori = (delta_ori * R.from_quat(robot_init_ori)).as_quat()  # [x, y, z, w]

        # Publish
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.pose.position.x = new_pos[0]
        msg.pose.position.y = new_pos[1]
        msg.pose.position.z = new_pos[2]
        msg.pose.orientation.x = new_ori[0]
        msg.pose.orientation.y = new_ori[1]
        msg.pose.orientation.z = new_ori[2]
        msg.pose.orientation.w = new_ori[3]
        self.relative_publisher.publish(msg)

    def publish_pose(self, position, orientation):
        # # Filter position and orientation
        # position = self.filter_value(position, key="pose_pos")
        # orientation = self.filter_value(orientation, key="pose_ori", is_orientation=True)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]
        self.publisher_.publish(msg)

    def publish_twist(self, current_pos, current_ori):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_tracker_pos is None or self.prev_tracker_ori is None or self.prev_time is None:
            self.prev_tracker_pos = current_pos
            self.prev_tracker_ori = current_ori
            self.prev_time = now
            print("Initializing previous tracker position and orientation.")
            return

        dt = now - self.prev_time
        if dt <= 0.005 or dt > 0.2:
            print(f"Skipping update: dt={dt:.4f} too small or too large")
            return

        # Filter position and orientation
        current_pos = self.filter_value(current_pos, key="twist_pos")
        current_ori = self.filter_value(current_ori, key="twist_ori", is_orientation=True)

        linear = (current_pos - self.prev_tracker_pos) / dt

        r_prev = R.from_quat(self.prev_tracker_ori)
        r_curr = R.from_quat(current_ori)
        r_rel = r_curr * r_prev.inv()
        rotvec = r_rel.as_rotvec()
        angular = rotvec / dt

        max_lin = 2.0
        max_ang = 6.0
        linear = np.clip(linear, -max_lin, max_lin)
        angular = np.clip(angular, -max_ang, max_ang)

        if hasattr(self, "prev_linear"):
            linear = self.alpha * linear + (1 - self.alpha) * self.prev_linear
            angular = self.alpha * angular + (1 - self.alpha) * self.prev_angular

        self.prev_linear = linear
        self.prev_angular = angular

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
        self.twist_publisher.publish(msg)

        self.prev_tracker_pos = current_pos
        self.prev_tracker_ori = current_ori
        self.prev_time = now

    def smooth(self, new, old):
        return self.alpha * new + (1 - self.alpha) * old

    def filter_value(self, new, key, is_orientation=False):
        """
        Exponential Moving Average filter for positions or orientations.
        Stores previous filtered values in self._filter_state[key].
        If is_orientation is True, uses SLERP for quaternion smoothing.
        """
        if key not in self._filter_state:
            self._filter_state[key] = new
            return new

        if is_orientation:
            from scipy.spatial.transform import Slerp
            times = [0, 1]
            key_rots = R.from_quat([self._filter_state[key], new])
            slerp = Slerp(times, key_rots)
            filtered = slerp(self.alpha).as_quat()
        else:
            filtered = self.alpha * new + (1 - self.alpha) * self._filter_state[key]

        self._filter_state[key] = filtered
        return filtered

# --- Main Async Loop ---
async def main(visualize=True, optimizer='adapted'):
    # UDP setup
    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005
    loop = asyncio.get_running_loop()
    vive_protocol = UDPReceiverProtocol()
    await loop.create_datagram_endpoint(
        lambda: vive_protocol,
        local_addr=(UDP_IP, UDP_PORT),
    )

    vis = None

    try:
        multiprocessing.set_start_method('spawn', force=True)
    except RuntimeError:
        # start method already set
        pass

    # ROS2 setup
    rclpy.init()
    node = ViveToROS2Publisher()

    # Camera setup (multiprocessing)
    queue = multiprocessing.Queue(maxsize=1)
    camera_path = "/dev/hand"  # or set to your camera device
    d405_serial_number = "218622273562"  # Replace with your D405 serial

    if camera_type == "realsense":
        producer_process = multiprocessing.Process(target=produce_realsense_frame, args=(queue, d405_serial_number, visualize))
    elif camera_type == "generic":
        producer_process = multiprocessing.Process(target=produce_camera_frame, args=(queue, camera_path, visualize))
    else:
        print(f"[ERROR] Unknown camera type: {camera_type}. Use 'generic' or 'realsense'.")
        return

    producer_process.start()

    # LEAP Hand setup
    if teleop_mode == "side_to_side":
        if optimizer == 'adapted':
            config_path = Path(get_package_share_directory('dex_retargeting')) / 'configs/teleop/leap_hand_right_dexpilot_adapted.yml'
        elif optimizer == 'original':
            config_path = Path(get_package_share_directory('dex_retargeting')) / 'configs/teleop/leap_hand_right_dexpilot.yml'
        # config_path = Path(__file__).resolve().parents[2] / "dex_retargeting/configs/teleop/leap_hand_right_dexpilot.yml"
        # config_path = Path('/home/user/franka_ros2_ws/src/dex_retargeting/src/dex_retargeting/configs/teleop/leap_hand_right_dexpilot.yml')
    elif teleop_mode == "mirror":
        config_path = Path(get_package_share_directory('dex_retargeting')) / 'configs/teleop/leap_hand_left_dexpilot.yml' 
        # config_path = Path(__file__).resolve().parents[2] / "dex_retargeting/configs/teleop/leap_hand_left_dexpilot.yml"
        # config_path = Path('/home/user/franka_ros2_ws/src/dex_retargeting/src/dex_retargeting/configs/teleop/leap_hand_left_dexpilot.yml')
    else:
        print(f"[ERROR] Unknown teleop mode: {teleop_mode}. Use 'mirror' or 'side_to_side'.")
        return
    
    if hand == True and ee_id == "leap_hand":
        robot_dir = Path(get_package_share_directory('dex_retargeting')) / 'assets/robots/hands'
        # robot_dir = Path(__file__).resolve().parents[2] / "assets/robots/hands"
        # robot_dir = Path('/home/user/franka_ros2_ws/src/dex_retargeting/assets/robots/hands')
        shutdown_event = multiprocessing.Event()
        consumer_process = multiprocessing.Process(
            target=start_retargeting,
            args=(queue, str(robot_dir), str(config_path), shutdown_event)
        )
        consumer_process.start()
    else:
        consumer_process = None
        print(f"[INFO] LEAP hand control is disabled (ee_id: {ee_id})")


    loop_counter = 0
    print_interval = 100  # Print every 100 loops
    last_print_time = time.time()
    target_dt = 1.0 / target_rate

    try:
        while rclpy.ok():
            loop_start = time.time()
            loop_counter += 1
            
            # --- Process Vive tracker data ---
            vive_poses = vive_protocol.poses
            # print(f"[DEBUG] Received {len(vive_poses)} VIVE poses", flush=True)
            if vive_poses and arm_control:
                # Example: Use the first tracker as the end-effector pose
                pose = vive_poses[0]
                
                # Convert position from VIVE to ROS (MoveIt) coordinates
                # VIVE: x (right), y (up), z (backward)
                # ROS:  x (forward), y (left), z (up)

                if teleop_mode == "side_to_side":
                    position = np.array([
                        -pose["position"]["z"],  # ROS X = -VIVE Z (forward/backward)
                        -pose["position"]["x"],  # ROS Y = -VIVE X (left/right)
                        pose["position"]["y"]    # ROS Z = VIVE Y (up/down)
                    ])

                    # Convert orientation (quaternion) from VIVE to ROS
                    # VIVE quaternion: [w, x, y, z]
                    orientation = np.array([
                        -pose["orientation"]["z"],
                        -pose["orientation"]["x"],
                        pose["orientation"]["y"],
                        pose["orientation"]["w"]
                    ])

                elif teleop_mode == "mirror":
                    position = np.array([
                        pose["position"]["z"],  # ROS X = VIVE Z (forward/backward)
                        -pose["position"]["x"],  # ROS Y = -VIVE X (left/right)
                        pose["position"]["y"]    # ROS Z = VIVE Y (up/down)
                    ])

                    # Convert orientation (quaternion) from VIVE to ROS
                    # VIVE quaternion: [w, x, y, z]
                    orientation = np.array([
                        pose["orientation"]["z"],
                        -pose["orientation"]["x"],
                        -pose["orientation"]["y"],
                        pose["orientation"]["w"]
                    ])
                else:
                    print(f"[ERROR] Unknown teleop mode: {teleop_mode}. Use 'mirror' or 'side_to_side'.")
                    continue

                # # Define the rotation from VIVE to ROS axes
                # # Example: 90 deg about Y then 90 deg about Z (adjust as needed)
                # # This is a common mapping for VIVE to ROS, but test for your setup!
                # R_vive_to_ros = R.from_euler('zyx', [0,0,0])
                # R_vive = R.from_quat(q_vive)  # [x, y, z, w]
                # R_ros = R_vive_to_ros * R_vive
                # q_ros = R_ros.as_quat()  # [x, y, z, w]

                # orientation = np.array([
                #     q_ros[0],  # x
                #     q_ros[1],  # y
                #     q_ros[2],  # z
                #     q_ros[3],  # w
                # ])

                # print(f"VIVE Pose: {pose}")
                # print(f"ROS Position: {position}")
                # print(f"ROS Orientation: {orientation}")

                node.current_tracker_pos = position
                node.current_tracker_ori = orientation
                node.publish_pose(position, orientation)
                if node.tracking_enabled:
                    node.publish_relative_pose(position, orientation)
                # node.publish_twist(position, orientation)
                
            elif not arm_control:
                if loop_counter % print_interval == 0:
                    print("[DEBUG] Arm control disabled, publishing dummy pose", flush=True)
                position = np.array([0.0, 0.0, 0.0])  # Dummy position
                orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Dummy orientation (identity quaternion)
                node.current_tracker_pos = position
                node.current_tracker_ori = orientation
                node.publish_pose(position, orientation)
                if node.tracking_enabled:
                    node.publish_relative_pose(position, orientation)

            else:
                if loop_counter % print_interval == 0:
                    print("[DEBUG] No VIVE poses received, skipping this loop", flush=True)
                pass
            
            rclpy.spin_once(node, timeout_sec=0.01)

            # # NEW: also spin LeapNode so it handles the service
            # if leap_hand is not None:
            #     rclpy.spin_once(leap_hand, timeout_sec=0.01)

            # --- Visualization ---
            if visualize and vive_poses:
                if vis is None:
                    vis = o3d.visualization.Visualizer()
                    vis.create_window()
                update_coordinate_frames(vive_poses, vis)
                # await asyncio.sleep(0.01)  # 30 FPS for visualization

            # --- Print actual loop rate ---
            if loop_counter % print_interval == 0:
                now = time.time()
                elapsed = now - last_print_time
                rate = print_interval / elapsed if elapsed > 0 else 0
                print(f"[INFO] Actual publishing rate: {rate:.2f} Hz over last {print_interval} loops", flush=True)
                last_print_time = now

            # --- Sleep only for the remaining time to maintain target rate ---
            loop_end = time.time()
            elapsed_loop = loop_end - loop_start
            sleep_time = max(0, target_dt - elapsed_loop)
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                print(f"[WARNING] Loop took too long: {elapsed_loop:.4f} seconds, skipping sleep", flush=True)
    
    except asyncio.CancelledError:
        print("[INFO] Shutdown requested via asyncio.CancelledError.")
    except KeyboardInterrupt:
        print("[INFO] Keyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"[ERROR] An error occurred: {e}", flush=True)
        logger.exception("Exception in main loop")
    finally:
        # 1) signal child processes to stop
        try:
            shutdown_event.set()
        except Exception:
            pass

        # 2) give consumer a short grace period to exit
        if consumer_process is not None:
            consumer_process.join(timeout=2.0)
            if consumer_process.is_alive():
                # force-kill only if needed
                consumer_process.terminate()
                consumer_process.join(timeout=1.0)

        # 3) stop producer (camera) more gracefully if possible
        try:
            producer_process.terminate()
            producer_process.join(timeout=1.0)
        except Exception:
            pass

        # 4) destroy visual windows etc.
        try:
            cv2.destroyAllWindows()
            if visualize and vis is not None:
                vis.destroy_window()
        except Exception:
            pass

        # 5) Now safe to shutdown rclpy in parent
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def run():
    parser = argparse.ArgumentParser(description="Teleoperate a Franka robot using VIVE trackers and LEAP Hand with hand tracking.")
    parser.add_argument(
        "-v", "--visualize",
        type=str,
        default="true",
        help="Visualize the camera feed and VIVE trackers (true/false)"
    )
    parser.add_argument(
        "-o", "--optimizer",
        type=str,
        default="adapted",
        help="Retargeting optimizer to use (e.g., 'adapted', 'original', etc.)"
    )
    args = parser.parse_args()

    asyncio.run(main(args.visualize.lower() == "true", args.optimizer))

if __name__ == "__main__":
    run()