import rclpy
from rclpy.node import Node
import numpy as np
import torch
import pickle
from pathlib import Path
import time
import yaml
import os
import logging
import inquirer
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
import asyncio
import threading

# ROS 2 message types
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Direct Hardware API Imports (from your recorder)
import pyrealsense2 as rs
from tact9d.shape_reconstruction import Sensor
from leap_hand.srv import LeapPosition, LeapPosVelEff

# Your ML Pipeline Imports
from model_pipeline.train import build_model, MLPPolicy
from model_pipeline.tactile_features import process_tactile_image
from model_pipeline.keypoint_extractor import KeypointExtractor
from model_pipeline.visual_embedder import VisualEmbedder

from model_pipeline import paths

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

SENSOR_ORDER = ["rindex", "rmiddle", "rthumb"]
TACT_BASE_PATH = str(Path(get_package_share_directory('tact9d')) / 'shape_reconstruction')

class PolicyRolloutNode(Node):
    def __init__(self):
        super().__init__('policy_rollout_node')

        # --- State Flags ---
        self.dependencies_ready = False
        
        # --- Declare ROS Parameters (for non-interactive use) ---
        self.declare_parameter('model_path', '')
        self.declare_parameter('goal_state_path', '')
        self.declare_parameter('control_rate_hz', 20.0)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # --- Load Model, Config, and Goal from provided paths ---
        with open(paths.DEFAULT_CONFIG_PATH, 'r') as f:
            self.config = yaml.safe_load(f)

        model_path_str = self.get_parameter('model_path').get_parameter_value().string_value
        goal_path_str = self.get_parameter('goal_state_path').get_parameter_value().string_value

        if not model_path_str or not goal_path_str:
            self.get_logger().fatal("CRITICAL: 'model_path' or 'goal_state_path' not provided. Shutting down.")
            raise ValueError("Missing required parameters.")
        else:
            self.get_logger().info(f"Loading model from: {model_path_str}")
            self.get_logger().info(f"Loading goal state from: {goal_path_str}")

        checkpoint = torch.load(model_path_str, map_location=self.device, weights_only=False)
        self.frame_stack_k = checkpoint.get("frame_stack", 1)
        self.is_arm_only = checkpoint.get("arm_only", False)
        self.num_arm_joints = checkpoint.get("num_arm_joints", 7)
        model_type = checkpoint["model_type"]
        
        inferred_width = checkpoint['state_dict']['net.0.bias'].shape[0] if model_type == 'mlp' else None
        self.model = build_model(model_type, checkpoint["input_dim"], checkpoint["output_dim"], width=inferred_width).to(self.device).eval()
        self.model.load_state_dict(checkpoint["state_dict"])
        
        self.X_mean = torch.tensor(checkpoint["X_mean"], device=self.device)
        self.X_std = torch.tensor(checkpoint["X_std"], device=self.device)
        self.y_mean = torch.tensor(checkpoint["y_mean"], device=self.device)
        self.y_std = torch.tensor(checkpoint["y_std"], device=self.device)

        with open(paths.WORKSPACE_ROOT / goal_path_str, 'rb') as f:
            self.goal_state = torch.from_numpy(pickle.load(f)).float().to(self.device)

        self.get_logger().info(f"Loaded model from {model_path_str} (K={self.frame_stack_k}, arm_only={self.is_arm_only})")
        self.get_logger().info(f"Loaded goal state from {goal_path_str} (Shape: {self.goal_state.shape})")

        # --- Initialize Hardware APIs (Mirrors fr3_leap_recorder.py) ---
        self.vision_processor = self.initialize_vision_processor()
        self.tactile_sensors = self.initialize_tactile_sensors()
        self.realsense = self.initialize_realsense()
        
        # --- ROS 2 Interfaces ---
        self.joint_state_sub = self.create_subscription(JointState, '/franka/joint_states', self.joint_state_callback, 10)
        self.command_pub = self.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10)
        
        # --- LEAP Hand Service Client ---
        self.leap_position_client = self.create_client(LeapPosition, '/leap_position')
        # # Wait for the service to be available
        # while not self.leap_position_client.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().info('Waiting for /leap_position service...')
        # self.get_logger().info("Connected to /leap_position service.")

        self.current_joint_states = None
        self.current_arm_states = None
        self.full_joint_names = None
        self.joint_names = [] 
        self.history_buffer = []
        self.last_known_positions = {'camera1': np.full(self.vision_processor.feature_dim_per_object, -1.0, dtype=np.float32),
                                     'camera2': np.full(self.vision_processor.feature_dim_per_object, -1.0, dtype=np.float32)}

        self.control_rate = self.get_parameter('control_rate_hz').get_parameter_value().double_value
        self.sample_period = 1.0 / self.control_rate
        self.get_logger().info(f"✅ Policy rollout node initialized. Running at {self.control_rate} Hz.")

    def initialize_vision_processor(self):
        vision_config = self.config.get("vision", {})
        data_dirs = [str(paths.RAW_DATA_DIR)] # For depth range calculation

        if vision_config.get("use_keypoint_extractor", False):
            logging.info("Initializing KeypointExtractor (YOLO) for visual features.")
            vision_module = KeypointExtractor(
                model_path=str(paths.WORKSPACE_ROOT / vision_config["yolo_model_path"]),
                confidence_threshold=vision_config.get("confidence_threshold", 0.1),
                use_3d=vision_config.get("use_3d", False),
                intrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("intrinsics_path_cam1")),
                extrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("extrinsics_path_cam1")),
                device=self.device
            )
            vision_module.is_keypoint_extractor = True
            return vision_module
        
        else: # Default to VisualEmbedder
            logging.info("Initializing VisualEmbedder (ResNet) for visual features.")
            global_depth_range = self.config.get("global_depth_range")
            if not global_depth_range:
                logging.warning("Global depth range not found, using default.")
                global_depth_range = (0, 1000)

            class ResNetWrapper:
                def __init__(self, config, device, depth_range):
                    self.embedder = VisualEmbedder(
                        backbone=config.get("backbone", "resnet18"), device=device,
                        out_dim={'rgb': config.get("visual_dim", 256), 'depth': config.get("depth_dim", 128)},
                        global_depth_range=depth_range
                    )
                    self.is_keypoint_extractor = False
                    self.feature_dim_per_object = self.embedder.out_dim['rgb'] + self.embedder.out_dim['depth']
                
                def extract_scene_features(self, color_img, depth_img):
                    if color_img is None: return np.zeros(self.feature_dim_per_object, dtype=np.float32)
                    rgb_emb = self.embedder.embed_rgb(color_img)
                    depth_emb = self.embedder.embed_depth(depth_img)
                    if rgb_emb is None or depth_emb is None: return np.zeros(self.feature_dim_per_object, dtype=np.float32)
                    return np.concatenate([rgb_emb, depth_emb])
            
            return ResNetWrapper(self.config, self.device, global_depth_range)

    def initialize_tactile_sensors(self):
        self.get_logger().info("Initializing 9DTact sensors...")
        sensors = {}
        configs = {'rthumb': "shape_config_thumb.yaml", 'rindex': "shape_config_index.yaml", 'rmiddle': "shape_config_middle.yaml"}
        for name, cfg_file in configs.items():
            cfg_path = os.path.join(TACT_BASE_PATH, cfg_file)
            with open(cfg_path, 'r') as f: cfg = yaml.safe_load(f)
            sensors[name] = Sensor(cfg)
        self.get_logger().info("9DTact sensors initialized.")
        return sensors

    def initialize_realsense(self):
        """
        FIXED: Initializes cameras using specific serial numbers for deterministic order,
        and stores pipelines/aligners with consistent keys ('camera1', 'camera2').
        """
        self.get_logger().info("Initializing RealSense cameras...")
        
        # Use the known serials from your recorder to guarantee order
        camera_serials = {
            'camera1': '151422254571',
            'camera2': '036522072607',
        }

        pipelines = {}
        aligners = {}
        
        ctx = rs.context()
        connected_devices = {dev.get_info(rs.camera_info.serial_number) for dev in ctx.query_devices()}

        for cam_id, serial in camera_serials.items():
            if serial not in connected_devices:
                self.get_logger().error(f"FATAL: RealSense camera with serial {serial} for {cam_id} not found.")
                raise RuntimeError(f"Camera {serial} not connected.")

            p = rs.pipeline()
            conf = rs.config()
            conf.enable_device(serial)
            conf.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            conf.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            p.start(conf)
            
            pipelines[cam_id] = p
            aligners[cam_id] = rs.align(rs.stream.color)
            self.get_logger().info(f"Successfully started {cam_id} (Serial: {serial}).")
        
        return {'pipelines': pipelines, 'aligners': aligners}

    def get_live_observations(self):
        """
        FIXED: Uses the correct dictionary keys ('camera1', 'camera2') to access frames.
        """
        try:
            obs = {}
            # Get RealSense data
            # Use the correct key 'camera1' for both pipelines and aligners
            frames1 = self.realsense['pipelines']['camera1'].wait_for_frames()
            aligned1 = self.realsense['aligners']['camera1'].process(frames1)
            obs['color1'] = np.asanyarray(aligned1.get_color_frame().get_data())
            obs['depth1'] = np.asanyarray(aligned1.get_depth_frame().get_data())
            
            # Use the correct key 'camera2' for both
            frames2 = self.realsense['pipelines']['camera2'].wait_for_frames()
            aligned2 = self.realsense['aligners']['camera2'].process(frames2)
            obs['color2'] = np.asanyarray(aligned2.get_color_frame().get_data())
            obs['depth2'] = np.asanyarray(aligned2.get_depth_frame().get_data())
            
            # Get Tactile data (this part was already correct)
            obs['tactile_images'] = {}
            for name, sensor in self.tactile_sensors.items():
                raw_img = sensor.get_rectify_crop_image()
                # --- CRITICAL FIX ---
                if raw_img is None:
                    self.get_logger().warn(f"Tactile sensor '{name}' returned a None image. Skipping observation.")
                    return None # Return None for the whole observation if any sensor fails
                obs['tactile_images'][name] = raw_img
                
            return obs
        except Exception as e:
            self.get_logger().warn(f"Failed to get live observations: {e}")
            return None
        
    def joint_state_callback(self, msg: JointState):
        """Callback for the Franka arm's joint states ONLY."""
        # This runs in the background, keeping the arm state fresh.
        if len(msg.position) >= 7:
            self.current_arm_states = np.array(msg.position[:7])
            if not self.full_joint_names: # Assume hand names are fixed
                self.full_joint_names = list(msg.name[:7]) + [f'joint{i+7}' for i in range(16)]

    def get_current_leap_position(self):
        """Calls the LEAP position service to get the current hand joint positions."""
        if not self.leap_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("LEAP position service not available.")
            return None

        req = LeapPosition.Request()
        future = self.leap_position_client.call_async(req)
        
        # This call is safe because the executor is running in a separate thread.
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5) 
        
        if future.done() and future.result() is not None:
            return np.array(future.result().position)
        else:
            self.get_logger().warn("Failed to get LEAP position from service (timed out or failed).")
            return None
                        
    def control_loop(self):
        """FIX: A regular, synchronous callback function."""
        self.get_logger().info("Waiting for initial arm states...")
        while rclpy.ok() and self.current_arm_states is None:
            time.sleep(0.1)
        
        self.get_logger().info("✅ All dependencies ready. Starting autonomous control loop.")

        while rclpy.ok():
            time_start = time.perf_counter()
            
            # --- Main Logic ---
            obs = self.get_live_observations()
            if obs is None: 
                time.sleep(self.sample_period)
                continue

            leap_positions = self.get_current_leap_position()
            if leap_positions is None: 
                time.sleep(self.sample_period)
                continue
            
            self.current_joint_states = np.concatenate([self.current_arm_states, leap_positions])        # 2. PREPARE STATE: Process sensor data into a feature vector
            tactile_feats = np.concatenate([
                process_tactile_image(
                    img=obs['tactile_images'][name],
                    use_height_map=True,
                    sensor=self.tactile_sensors[name],
                    ref_img=self.tactile_sensors[name].ref  # <-- The fix
                )[0] for name in SENSOR_ORDER
            ])            

            # Process each camera's data stream
            cam1_feats = self.vision_processor.extract_scene_features(obs['color1'], obs['depth1'])
            cam2_feats = self.vision_processor.extract_scene_features(obs['color2'], obs['depth2'])
            
            raw_features_per_cam = {'cam1': cam1_feats, 'cam2': cam2_feats}

            # Handle "Carry Forward" logic if using KeypointExtractor
            if self.vision_processor.is_keypoint_extractor:
                for cam_id, features in raw_features_per_cam.items():
                    if features is not None and np.all(features == 0): # Failed detection
                        if np.all(self.last_known_positions[cam_id] != -1.0):
                            raw_features_per_cam[cam_id] = self.last_known_positions[cam_id]
                    elif features is not None and not np.all(features == 0): # Successful detection
                        self.last_known_positions[cam_id] = features
            
            # Concatenate features into the final vector
            f1 = raw_features_per_cam.get('cam1', np.zeros(self.vision_processor.output_dim, dtype=np.float32))
            f2 = raw_features_per_cam.get('cam2', np.zeros(self.vision_processor.output_dim, dtype=np.float32))
            visual_features = np.concatenate([f1, f2])

            proprio_data = self.current_joint_states[:self.num_arm_joints] if self.is_arm_only else self.current_joint_states            
            current_state_np = np.concatenate([tactile_feats, visual_features, proprio_data])

            # --- DIAGNOSTIC PRINTS ---
            self.get_logger().info(f"--- Shape Analysis ---")
            self.get_logger().info(f"Tactile Features Shape: {tactile_feats.shape}")
            self.get_logger().info(f"Visual Features Shape:  {visual_features.shape}")
            self.get_logger().info(f"Proprio Shape:          {proprio_data.shape}")
            
            # This is the vector being created live
            current_state_np = np.concatenate([tactile_feats, visual_features, proprio_data])
            self.get_logger().info(f"Current State Vector Shape: {current_state_np.shape}")

            # The goal state's shape reflects the TRAINING data format
            self.get_logger().info(f"Goal State Vector Shape:    {self.goal_state.shape}")
            self.get_logger().info(f"------------------------")
            
            full_state_np = np.concatenate([current_state_np, self.goal_state.cpu().numpy()])
            
            # 3. HANDLE HISTORY (Frame Stacking)
            self.history_buffer.append(full_state_np)
            if len(self.history_buffer) > self.frame_stack_k:
                self.history_buffer.pop(0)
            if len(self.history_buffer) < self.frame_stack_k:
                self.get_logger().info(f"Filling history buffer... {len(self.history_buffer)}/{self.frame_stack_k}")
                continue

            state_sequence = torch.from_numpy(np.array(self.history_buffer)).float().to(self.device)
            
            # 4. INFER ACTION
            with torch.no_grad():
                is_sequence_model = not isinstance(self.model, MLPPolicy)
                if is_sequence_model:
                    state_norm = (state_sequence - self.X_mean) / self.X_std
                    state_norm = state_norm.unsqueeze(0)
                else: # MLP
                    state_norm = (state_sequence.flatten() - self.X_mean.repeat(self.frame_stack_k)) / self.X_std.repeat(self.frame_stack_k)
                    state_norm = state_norm.unsqueeze(0)
                
                action_norm = self.model(state_norm).squeeze(0)
                delta_q_pred = (action_norm * self.y_std) + self.y_mean

            # 5. PUBLISH COMMAND
            if self.is_arm_only:
                num_hand_joints = 23 - self.num_arm_joints
                hand_zeros = torch.zeros(num_hand_joints, device=self.device)
                full_delta_q = torch.cat([delta_q_pred, hand_zeros])
            else:
                full_delta_q = delta_q_pred
                
            current_q_torch = torch.from_numpy(self.current_joint_states).to(self.device)
            target_q = current_q_torch + full_delta_q

            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = target_q.cpu().tolist()
            point.time_from_start = Duration(sec=0, nanosec=int(1e9 / self.get_parameter('control_rate_hz').get_parameter_value().double_value))
            traj_msg.points.append(point)
            self.command_pub.publish(traj_msg)

            time_end = time.perf_counter()
            time_sleep = max(0, self.sample_period - (time_end - time_start))
            time.sleep(time_sleep)

def main(args=None):
    rclpy.init(args=args)

    node = PolicyRolloutNode()

    # --- FIX: A simple, standard main function ---
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    try:
        executor_thread.start()
        node.control_loop()  # Initial call to start the loop
        
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()