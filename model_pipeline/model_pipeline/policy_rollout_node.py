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
from std_msgs.msg import Float64MultiArray
import pinocchio as pin
import tempfile
import random

# ROS 2 message types
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped # For task-space control
from builtin_interfaces.msg import Duration

# Direct Hardware API Imports (from your recorder)
import pyrealsense2 as rs
from tact9d.shape_reconstruction import Sensor
from leap_hand.srv import LeapPosition, LeapPosVelEff
import dex_retargeting.leap_hand_utils.leap_hand_utils as lhu

# Your ML Pipeline Imports
from model_pipeline.train import build_model, MLPPolicy
from model_pipeline.dataset_builder import VisionProcessor, SENSOR_ORDER
from model_pipeline.tactile_features import process_tactile_image
# from model_pipeline.keypoint_extractor import KeypointExtractor
# from model_pipeline.visual_embedder import VisualEmbedder
from model_pipeline.kinematics import KinematicsSolver, get_urdf_string_from_xacro
from model_pipeline.evaluate_policy import to_np
# from model_pipeline.utils import get_cfg_path, init_sensor

from model_pipeline import paths

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

TACT_BASE_PATH = str(Path(get_package_share_directory('tact9d')) / 'shape_reconstruction')

class PolicyRolloutNode(Node):
    def __init__(self):
        super().__init__('policy_rollout_node')

        SEED = 42
        os.environ["PYTHONHASHSEED"] = str(SEED)
        random.seed(SEED)
        np.random.seed(SEED)
        torch.manual_seed(SEED)
        torch.cuda.manual_seed_all(SEED)
        torch.use_deterministic_algorithms(True, warn_only=True)
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True

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

        self.control_mode = self.config.get('control_mode', {})
        self.num_arm_actions = 6 if self.control_mode == 'task_space' else 7

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
        self.model_type = checkpoint["model_type"]

        # --- THE CRITICAL FIX: Read hyperparameters from the checkpoint ---
        model_hyperparams = checkpoint.get("model_hyperparams", {})
        logging.info(f"Loaded model hyperparameters: {model_hyperparams}")
        logging.info(f"Model was trained with K={self.frame_stack_k}, type='{self.model_type}'. Evaluating accordingly.")
        if self.is_arm_only:
            logging.info(f"Evaluating an ARM-ONLY model with {self.num_arm_joints} joints.")

        inferred_width = checkpoint['state_dict']['net.0.bias'].shape[0] if self.model_type == 'mlp' else None
        self.model = build_model(
            self.model_type, 
            checkpoint["input_dim"], 
            checkpoint["output_dim"],
            # Pass all saved hyperparameters. The build_model function
            # will only use the ones it needs (e.g., 'width' for MLP).
            **model_hyperparams 
        ).to(self.device)
        self.model.load_state_dict(checkpoint["state_dict"])
        self.model.eval()

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
        self.solver = self.initialize_kinematics_solver()
        
        # --- ROS 2 Interfaces ---
        self.policy_arm_joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        self.received_state_order = None 
        self.state_to_policy_indices = None 
        
        self.joint_state_sub = self.create_subscription(JointState, '/franka/joint_states', self.joint_state_callback, 10)
        if self.control_mode == "joint_space":
            self.arm_command_pub = self.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10)
        else:
            self.arm_command_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        # --- LEAP Hand Service Client ---
        self.leap_position_client = self.create_client(LeapPosition, '/leap_position')
        self.hand_command_pub = self.create_publisher(Float64MultiArray, '/leap_hand/target_allegro_pose', 10)

        # # Wait for the service to be available
        # while not self.leap_position_client.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().info('Waiting for /leap_position service...')
        # self.get_logger().info("Connected to /leap_position service.")

        self.current_joint_states = None
        self.current_arm_states = None
        self.full_joint_names = None
        self.history_buffer = []

        # --- Stateful Tracking for Keypoints ---
        # We now store the last known *position* vector separately from the *full feature* vector.
        # The (x,y,z) or (u,v) part of the feature.
        self.coord_dim = 3 if self.config.get('state', {}).get('use_3d_keypoints', False) else 2

        # --- Get dimensions from the initialized vision_processor ---
        # These are crucial for correctly parsing the feature vector
        self.kp_raw_dim = self.vision_processor.extractor1.output_dim  # e.g., 10 (5 per object)
        self.kp_engineered_dim = self.coord_dim
        self.kp_total_dim = self.kp_raw_dim + self.kp_engineered_dim # e.g., 13
        
        self.single_cam_dim = self.vision_processor.single_cam_dim # e.g., 61

        # Store the last known [x,y,z] or [u,v]
        self.last_known_coords = {
            'cam1_tube': np.zeros(self.coord_dim, dtype=np.float32),
            'cam1_peg':  np.zeros(self.coord_dim, dtype=np.float32),
            'cam2_tube': np.zeros(self.coord_dim, dtype=np.float32),
            'cam2_peg':  np.zeros(self.coord_dim, dtype=np.float32)
        }

        self.proprio_history = []
        self.prediction_history = []
        self.timestamp_history = []

        self.control_rate = self.get_parameter('control_rate_hz').get_parameter_value().double_value
        self.control_rate = self.control_rate /2  # Slower rollout for testing
        self.sample_period = 1.0 / self.control_rate

        logging.info("Commanding a slow, interpolated grasp...")
        start_pose_arm = self.config.get('robot_setup', {}).get('start_pose_arm')
        hand_grasp_pose = self.config.get('robot_setup', {}).get('hand_grasp_pose')

        open_hand_pose = lhu.allegro_to_LEAPhand([0.0]*16, zeros=False)
        # Define grasp parameters
        grasp_duration = 2.0  # Total time for the grasp in seconds
        num_steps = 100        # Number of intermediate steps
        hand_grasp_pose = np.array(hand_grasp_pose)

        # Loop to send intermediate poses
        for i in range(num_steps + 1):
            # Calculate the interpolation factor (alpha) from 0.0 to 1.0
            alpha = i / num_steps
            
            # Linearly interpolate between the open and closed poses
            intermediate_pose = (1 - alpha) * open_hand_pose + alpha * hand_grasp_pose
            
            # Send the command (convert back to list for the function)
            msg = Float64MultiArray()
            msg.data = [float(p) for p in intermediate_pose.tolist()]
            self.hand_command_pub.publish(msg)
            self.get_logger().info("Hand command sent.")

            # Wait for a short duration before the next step
            time.sleep(grasp_duration / num_steps)
        
        logging.info("Grasp complete.")
        time.sleep(10) # A final short pause after grasping
        
        self.get_logger().info(f"✅ Policy rollout node initialized. Running at {self.control_rate} Hz.")

    def initialize_vision_processor(self):
        data_dirs = [str(paths.RAW_DATA_DIR)] # For depth range calculation
        return VisionProcessor(self.config, self.device, data_dirs)
    
        # vision_config = self.config.get("vision", {})
        # state_config = self.config.get("state", {})

        # if state_config.get("use_keypoint_extractor", False):
        #     logging.info("Initializing KeypointExtractor (YOLO) for visual features.")
        #     vision_module = KeypointExtractor(
        #         model_path=str(paths.WORKSPACE_ROOT / vision_config["yolo_model_path"]),
        #         confidence_threshold=vision_config.get("confidence_threshold", 0.1),
        #         use_3d=vision_config.get("use_3d", False),
        #         intrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("intrinsics_path_cam1")),
        #         extrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("extrinsics_path_cam1")),
        #         device=self.device
        #     )
        #     vision_module.is_keypoint_extractor = True
        #     return vision_module
        
        # if state_config.get("use_embeddings", False): # Default to VisualEmbedder
        #     logging.info("Initializing VisualEmbedder (ResNet) for visual features.")
        #     global_depth_range = self.config.get("global_depth_range")
        #     if not global_depth_range:
        #         logging.warning("Global depth range not found, using default.")
        #         global_depth_range = (0, 1000)

        #     class ResNetWrapper:
        #         def __init__(self, config, device, depth_range):
        #             self.embedder = VisualEmbedder(
        #                 backbone=config.get("backbone", "resnet18"), device=device,
        #                 out_dim={'rgb': config.get("visual_dim", 256), 'depth': config.get("depth_dim", 128)},
        #                 global_depth_range=depth_range
        #             )
        #             self.is_keypoint_extractor = False
        #             self.feature_dim_per_object = self.embedder.out_dim['rgb'] + self.embedder.out_dim['depth']
                
        #         def extract_scene_features(self, color_img, depth_img):
        #             if color_img is None: return np.zeros(self.feature_dim_per_object, dtype=np.float32)
        #             rgb_emb = self.embedder.embed_rgb(color_img)
        #             depth_emb = self.embedder.embed_depth(depth_img)
        #             if rgb_emb is None or depth_emb is None: return np.zeros(self.feature_dim_per_object, dtype=np.float32)
        #             return np.concatenate([rgb_emb, depth_emb])
            
        #     return ResNetWrapper(self.config, self.device, global_depth_range)

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
    
    def initialize_kinematics_solver(self):
        urdf_content = get_urdf_string_from_xacro()
        if not urdf_content: 
            logging.error("Failed to generate URDF, cannot proceed."); return

        urdf_temp_file = None
        try:
            # 2b. Save to a temporary file for Pinocchio to load
            # This creates a file with a unique name in the system's temp directory
            with tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.urdf', encoding='utf-8') as f:
                f.write(urdf_content)
                urdf_temp_file = Path(f.name)
        
            kinematics_config = self.config.get("kinematics", {})
            self.solver = KinematicsSolver(
                urdf_content=urdf_temp_file,
                end_effector_frame_name=kinematics_config.get("ee_frame", "fr3_hand_tcp"),
                tactile_frame_names=kinematics_config.get("tactile_frames", []),
                visualize=False,
            )
        except Exception as e:
            logging.error(f"Error initializing KinematicsSolver: {e}")
            return
        finally:
            # Clean up the temporary URDF file
            if urdf_temp_file and urdf_temp_file.exists():
                urdf_temp_file.unlink()

        
    def joint_state_callback(self, msg: JointState):
        """Callback for the Franka arm's joint states ONLY."""
        
        if len(msg.position) < 7:
            return

        # --- Dynamic Mapping Check (Run only once) ---
        if self.received_state_order is None:
            # The first 7 names are the arm joints (in whatever order the publisher sends)
            self.received_state_order = msg.name[:7]
            
            # Create the index map: For each joint in our policy order, find its index 
            # in the received state order.
            self.state_to_policy_indices = np.array([
                self.received_state_order.index(name) 
                for name in self.policy_arm_joint_names
            ])
            # self.get_logger().info(f"Dynamically mapped arm joint order for policy input.")
            # self.get_logger().info(f"Received Order: {self.received_state_order}")
            # self.get_logger().info(f"Policy Order:   {self.policy_arm_joint_names}")
            # self.get_logger().info(f"Reorder Indices: {self.state_to_policy_indices}")


        # --- State Processing ---
        # 1. Take the received joint positions (in received order)
        arm_positions_received_order = np.array(msg.position[:7])
        
        # 2. Re-order the data using the computed index map
        # self.current_arm_states is now guaranteed to be in the self.policy_arm_joint_names order (J1-J7)
        self.current_arm_states = arm_positions_received_order[self.state_to_policy_indices]
        
        # --- Update full_joint_names (if needed for the policy input vector) ---
        # Assuming policy input requires all 23 joints in a specific order:
        if self.full_joint_names is None:
            # full_joint_names (23 joints) must be the order your POLICY expects
            # Example: [J1...J7, Hand_J1...Hand_J16]
            hand_names = msg.name[7:] # Get the rest of the joints (hand, etc.)
            self.full_joint_names = self.policy_arm_joint_names + hand_names

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
        """A regular, synchronous callback function."""
        self.get_logger().info("Waiting for initial arm states...")
        while rclpy.ok() and self.current_arm_states is None:
            time.sleep(0.1)

        # --- SAFETY INITIALIZATION ---
        # Initialize previous commands to zero to ensure a smooth start.
        # It's best practice to define these in your node's __init__ method.
        self.previous_arm_command = np.zeros(self.num_arm_actions)
        if not self.is_arm_only:
            self.previous_hand_command = np.zeros(16) 

        # --- SAFETY PARAMETERS (Tune these for your hardware!) ---
        self.MAX_ARM_DELTA = 0.05   # Absolute max delta (rad or m)
        self.ARM_RATE_LIMIT = 0.02  # Max change from the last command
        self.MAX_HAND_DELTA = 0.1   # Absolute max delta for hand joints
        self.HAND_RATE_LIMIT = 0.05 # Max change for hand commands
        # ---`--------------------------------------------------------
        
        self.get_logger().info("✅ All dependencies ready. Starting autonomous control loop.")

        try:
            while rclpy.ok():
                time_start = time.perf_counter()
                
                # --- 1. SENSE ---
                obs = self.get_live_observations()
                if obs is None: 
                    time.sleep(self.sample_period)
                    continue

                leap_positions = self.get_current_leap_position()
                if leap_positions is None: 
                    time.sleep(self.sample_period)
                    continue
                
                self.current_joint_states = np.concatenate([self.current_arm_states, leap_positions])

                # --- 2. PREPARE STATE (Mirrors dataset_builder.py) ---
                state_vector_list = []
                current_proprio = None

                # 2.1 Tactile Features
                tactile_feats = np.concatenate([
                    process_tactile_image(
                        img=obs['tactile_images'][name],
                        use_height_map=True,
                        sensor=self.tactile_sensors[name],
                        ref_img=self.tactile_sensors[name].ref
                    )[0] for name in SENSOR_ORDER
                ])            
                state_vector_list.append(tactile_feats)

                # 2.2 Proprioceptive Features
                if self.control_mode == 'task_space':
                    kinematic_poses = self.solver.get_all_poses(self.current_arm_states, leap_positions)
                    arm_proprio = kinematic_poses['ee'] # 7D EE Pose
                    state_vector_list.append(arm_proprio)
                else: # joint_space
                    arm_proprio = self.current_arm_states # 7D Arm Joints
                    state_vector_list.append(arm_proprio)            

                hand_proprio = leap_positions # 16D Hand Joints
                state_vector_list.append(hand_proprio)
                current_proprio = np.concatenate([arm_proprio, hand_proprio])

                # 2.3 3D Tactile Poses
                if self.config.get('state', {}).get('use_3d_tactile'):
                    for frame_name in self.config['kinematics']['tactile_frames']:
                        state_vector_list.append(kinematic_poses[frame_name])

                # 2.4 Visual Features (with Carry-Forward Logic)
                features_per_cam = self.vision_processor.process_cameras(obs['color1'], obs['depth1'], obs['color2'], obs['depth2'])
                
                # --- "Carry Forward" Logic with Staleness Flag ---
                def update_features(raw_feats, cam_id):
                    """
                    Parses a 'kitchen sink' vector, applies staleness logic to the
                    keypoint part, and preserves the embedding part.
                    """
                    if raw_feats is None: # Handle case where a camera failed
                        return np.zeros(self.single_cam_dim, dtype=np.float32)

                    # --- 1. Split the "kitchen sink" vector ---
                    kps_vec = raw_feats[:self.kp_total_dim]
                    embs_vec = raw_feats[self.kp_total_dim:]
                    
                    # Deconstruct the keypoint vector
                    # [tube(5), peg(5), rel(3)]
                    tube_feats = kps_vec[0:self.coord_dim+2]
                    peg_feats = kps_vec[self.coord_dim+2:2*self.coord_dim+4]
                    
                    tube_key, peg_key = f"{cam_id}_tube", f"{cam_id}_peg"

                    # --- 2. Apply Staleness Logic ---
                    # Check TUBE detection (flag is at index 4)
                    if tube_feats[self.coord_dim+1] > 0: # if flag is 1
                        self.last_known_coords[tube_key] = tube_feats[0:self.coord_dim] # Store new 3D pos
                    else:
                        tube_feats[0:self.coord_dim] = self.last_known_coords[tube_key] # Use stale position
                        tube_feats[self.coord_dim:self.coord_dim+2] = 0.0 # Set conf and flag to 0

                    # Check PEG detection (flag is at index 4 of its vector)
                    if peg_feats[self.coord_dim+1] > 0: # if flag is 1
                        self.last_known_coords[peg_key] = peg_feats[0:self.coord_dim]
                    else:
                        peg_feats[0:self.coord_dim] = self.last_known_coords[peg_key]
                        peg_feats[self.coord_dim:self.coord_dim+2] = 0.0
                    
                    # --- 3. Re-engineer the relative vector ---
                    # This is crucial: we recalculate the relative vector based on the
                    # (potentially stale) coordinates we are passing to the policy.
                    rel_vec = tube_feats[0:self.coord_dim] - peg_feats[0:self.coord_dim]
                    
                    # Reconstruct the final keypoint vector
                    final_kps_vec = np.concatenate([tube_feats, peg_feats, rel_vec])
                    
                    # --- 4. Recombine with embeddings ---
                    # Return the full, corrected "kitchen sink" vector
                    return np.concatenate([final_kps_vec, embs_vec])

                # Get the raw "kitchen sink" vectors
                f1_raw = features_per_cam.get('cam1', np.zeros(self.single_cam_dim, dtype=np.float32))
                f2_raw = features_per_cam.get('cam2', np.zeros(self.single_cam_dim, dtype=np.float32))

                # Apply the staleness logic
                f1_final = update_features(f1_raw, 'cam1')
                f2_final = update_features(f2_raw, 'cam2')
                
                visual_features = np.concatenate([f1_final, f2_final])
                state_vector_list.append(visual_features)

                # --- Final State Assembly ---
                current_state_np = np.concatenate(state_vector_list)
                full_state_np = np.concatenate([current_state_np, self.goal_state.cpu().numpy()])

                # --- DIAGNOSTIC PRINTS ---
                # self.get_logger().info(f"--- Shape Analysis ---")
                # self.get_logger().info(f"Tactile Features Shape: {tactile_feats.shape}")
                tactile_log_msg = "Tactile Contact: "
                for i, name in enumerate(SENSOR_ORDER):
                    flag = tactile_feats[i * 8 + 7]  # Correctly checks the flag at the end of each sensor's 8 features
                    tactile_log_msg += f"✅ {name.upper()} | " if flag > 0 else f"❌ {name.upper()} | "
                self.get_logger().info(tactile_log_msg)

                # self.get_logger().info(f"Visual Features Shape:  {visual_features.shape}")
                visual_log_msg = "Visual Detections: "
                # 'raw_features_per_cam' now contains the engineered 10-element vectors
                for cam_id, feats in features_per_cam.items():
                    if feats is None: continue
                    tube_flag = feats[3]
                    peg_flag = feats[7]
                    visual_log_msg += f"{cam_id} [Tube: {'✅' if tube_flag > 0 else '❌'}, Peg: {'✅' if peg_flag > 0 else '❌'}] | "
                self.get_logger().info(visual_log_msg)

                # self.get_logger().info(f"Proprio Shape:          {proprio_data.shape}")
                # self.get_logger().info(f"Current State Vector Shape: {current_state_np.shape}")
                # self.get_logger().info(f"Goal State Vector Shape:    {self.goal_state.shape}")
                full_state_np = np.concatenate([current_state_np, self.goal_state.cpu().numpy()])
                
                # 3. HANDLE HISTORY (Frame Stacking)
                self.history_buffer.append(full_state_np)
                if len(self.history_buffer) > self.frame_stack_k:
                    self.history_buffer.pop(0)
                if len(self.history_buffer) < self.frame_stack_k:
                    self.get_logger().info(f"Filling history buffer... {len(self.history_buffer)}/{self.frame_stack_k}")
                    continue
                self.get_logger().info(f"------------------------")

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
                    action_pred = (action_norm * self.y_std) + self.y_mean

                if current_proprio is not None:
                    self.timestamp_history.append(time_start)
                    self.proprio_history.append(current_proprio)
                    self.prediction_history.append(action_pred.cpu().numpy())

                # 5. PUBLISH COMMAND
                action_pred_np = to_np(action_pred)

                # --- 5.1 Process Arm Command ---
                raw_delta_arm = action_pred_np[:self.num_arm_actions]

                # Step A: Clipping (Saturation)
                # Enforces a hard limit on the command's magnitude.
                clipped_delta_arm = np.clip(raw_delta_arm, -self.MAX_ARM_DELTA, self.MAX_ARM_DELTA)

                # Step B: Rate Limiting (Slew Rate Limiter)
                # Ensures a smooth ramp-up from the previous command. This is KEY.
                change_arm = clipped_delta_arm - self.previous_arm_command
                limited_change_arm = np.clip(change_arm, -self.ARM_RATE_LIMIT, self.ARM_RATE_LIMIT)
                safe_delta_arm = self.previous_arm_command + limited_change_arm
                
                # Update the state for the next cycle's rate limiter
                self.previous_arm_command = safe_delta_arm

                # --- Diagnostic Logging for Arm ---
                was_limited = not np.allclose(raw_delta_arm, safe_delta_arm, atol=1e-6)
                limit_status = "⚠️ CLIPPED/LIMITED" if was_limited else "✅ OK"
                self.get_logger().info(
                    f"ARM CMD | Status: {limit_status} | "
                    f"Raw: {np.round(raw_delta_arm, 4)} -> "
                    f"Safe: {np.round(safe_delta_arm, 4)}"
                )

                # --- Publish Arm Command ---
                if self.control_mode == 'task_space':
                    current_ee_pose = kinematic_poses['ee']
                    T_current = pin.XYZQUATToSE3(current_ee_pose)
                    # Use the SAFE command to compute the next pose
                    T_delta = pin.exp(safe_delta_arm)
                    T_next = T_current * T_delta
                    target_pose_7d = pin.se3ToXYZQUAT(T_next)
                    
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "base"
                    pose_msg.pose.position.x = target_pose_7d[0]
                    pose_msg.pose.position.y = target_pose_7d[1]
                    pose_msg.pose.position.z = target_pose_7d[2]
                    pose_msg.pose.orientation.x = target_pose_7d[3]
                    pose_msg.pose.orientation.y = target_pose_7d[4]
                    pose_msg.pose.orientation.z = target_pose_7d[5]
                    pose_msg.pose.orientation.w = target_pose_7d[6]
                    self.arm_command_pub.publish(pose_msg)

                else: # joint_space
                    # target_q_arm is in the CANONICAL (J1-J7) order. NO REORDERING NEEDED.
                    target_q_arm = self.current_arm_states + safe_delta_arm
                    
                    traj_msg = JointTrajectory()
                    
                    traj_msg.joint_names = self.policy_arm_joint_names # Use the known canonical order
                    
                    point = JointTrajectoryPoint()
                    point.positions = target_q_arm.tolist()
                    
                    # Use a safer duration (e.g., 0.5s)
                    duration_ns = int(1e9/2/self.control_rate)
                    point.time_from_start = Duration(sec=0, nanosec=duration_ns)
                    
                    traj_msg.points.append(point)
                    self.arm_command_pub.publish(traj_msg)

                # --- 5.2 Process Hand Command (with same safety logic) ---
                if not self.is_arm_only:
                    raw_delta_hand = action_pred_np[self.num_arm_actions:]
                    
                    # Apply clipping and rate limiting to the hand
                    clipped_delta_hand = np.clip(raw_delta_hand, -self.MAX_HAND_DELTA, self.MAX_HAND_DELTA)
                    change_hand = clipped_delta_hand - self.previous_hand_command
                    limited_change_hand = np.clip(change_hand, -self.HAND_RATE_LIMIT, self.HAND_RATE_LIMIT)
                    safe_delta_hand = self.previous_hand_command + limited_change_hand
                    
                    # Update for the next cycle
                    self.previous_hand_command = safe_delta_hand

                    # --- Diagnostic Logging for Hand ---
                    was_limited_hand = not np.allclose(raw_delta_hand, safe_delta_hand, atol=1e-6)
                    limit_status_hand = "⚠️ CLIPPED/LIMITED" if was_limited_hand else "✅ OK"
                    self.get_logger().info(
                        f"HAND CMD | Status: {limit_status_hand} | "
                        f"Raw: {np.round(raw_delta_hand[:4], 4)}... -> " # Log first 4 joints for brevity
                        f"Safe: {np.round(safe_delta_hand[:4], 4)}..."
                    )
                    # Use the SAFE hand command
                    target_q_hand_allegro = self.current_joint_states[7:] + safe_delta_hand
                    hand_msg = Float64MultiArray()
                    hand_msg.data = [float(p) for p in target_q_hand_allegro]
                    self.hand_command_pub.publish(hand_msg)

                time_end = time.perf_counter()
                time_sleep = max(0, self.sample_period - (time_end - time_start))
                time.sleep(time_sleep)

        except KeyboardInterrupt:
            logging.info("Rollout interrupted by user.")

        finally:
            self.save_trajectory_data()

    def save_trajectory_data(self):
        """Saves the collected trajectory data to a compressed numpy file."""
        if not self.proprio_history:
            self.get_logger().info("No trajectory data to save.")
            return

        self.get_logger().info(f"💾 Saving trajectory data to rollout_data.npz...")
        
        # Use np.stack to convert list of arrays into a single large array
        np.savez_compressed(
            str(paths.MODELS_DIR/f"rollout/rollout_{self.model_type}_{self.control_mode}.npz"),
            timestamps=np.array(self.timestamp_history),
            proprioception=np.stack(self.proprio_history),
            predictions=np.stack(self.prediction_history)
        )
        self.get_logger().info("✅ Trajectory data saved successfully.")

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