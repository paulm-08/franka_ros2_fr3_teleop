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
from threading import Lock
import pinocchio as pin
import tempfile
import random
import cv2

# ROS 2 message types
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointJog
from std_msgs.msg import Float64MultiArray, Header
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
        self.declare_parameter('control_rate_hz', 10.0)

        # --- REPLAY_MODE Parameters ---
        self.declare_parameter('replay_gt', False)
        self.declare_parameter('dataset_pkl', 'data/processed_datasets/dataset_single_test_noemb.pkl')
        self.declare_parameter('replay_traj_idx', 0)
        self.declare_parameter('start_step', 49)


        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # --- Load Model, Config, and Goal from provided paths ---
        model_path_str = self.get_parameter('model_path').get_parameter_value().string_value
        checkpoint = torch.load(model_path_str, map_location=self.device, weights_only=False)
        training_config = checkpoint.get("training_config", {})

        if training_config == {}:
            self.get_logger().info("WARNING: 'training_config' not found in checkpoint. Loading from default path.")
            with open(training_config, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = training_config        

        self.control_mode = self.config.get('control_mode', {})
        self.num_arm_actions = 6 if self.control_mode == 'task_space' else 7

        # --- REPLAY_MODE Flag ---
        self.is_replay_mode = self.get_parameter('replay_gt').get_parameter_value().bool_value
        self.replay_actions = None
        self.start_step = self.get_parameter('start_step').get_parameter_value().integer_value

        goal_path_str = self.get_parameter('goal_state_path').get_parameter_value().string_value

        if not model_path_str or not goal_path_str:
            self.get_logger().fatal("CRITICAL: 'model_path' or 'goal_state_path' not provided. Shutting down.")
            raise ValueError("Missing required parameters.")
        else:
            self.get_logger().info(f"Loading model from: {model_path_str}")
            self.get_logger().info(f"Loading goal state from: {goal_path_str}")

        self.frame_stack_k = checkpoint.get("frame_stack", 1)
        self.is_arm_only = checkpoint.get("arm_only", False)
        self.num_arm_joints = checkpoint.get("num_arm_joints", 7)
        self.model_type = checkpoint["model_type"]
        self.use_goal = checkpoint["training_config"].get("use_goal", False)

        # --- Read hyperparameters from the checkpoint ---
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
        if self.use_goal:
            self.get_logger().info(f"Loaded goal state from {goal_path_str} (Shape: {self.goal_state.shape})")
        else:
            self.get_logger().info(f"Loaded goal state, not using.")

        # --- Initialize Hardware APIs (Mirrors fr3_leap_recorder.py) ---
        self.vision_processor = self.initialize_vision_processor()
        self.tactile_sensors = self.initialize_tactile_sensors()
        self.realsense = self.initialize_realsense()
        self.solver = self.initialize_kinematics_solver()

        if self.is_replay_mode:
            # --- REPLAY MODE: Load Dataset Actions ---
            self.get_logger().info("--- ⚠️ REPLAY MODE ENABLED ⚠️ ---")
            dataset_pkl_path = self.get_parameter('dataset_pkl').get_parameter_value().string_value
            traj_idx = self.get_parameter('replay_traj_idx').get_parameter_value().integer_value
            
            if not dataset_pkl_path:
                self.get_logger().fatal("CRITICAL: 'replay_gt=True' but 'dataset_pkl' not provided.")
                raise ValueError("Missing required 'dataset_pkl' parameter.")

            try:
                with open(paths.WORKSPACE_ROOT / dataset_pkl_path, 'rb') as f:
                    all_trajectories = pickle.load(f)
                
                self.replay_actions = all_trajectories[traj_idx]['action_t']
                self.get_logger().info(f"Loaded GT actions from trajectory {traj_idx} in {dataset_pkl_path}")
                self.get_logger().info(f"Total actions loaded: {len(self.replay_actions)}. Starting from step: {self.start_step}")
                
                # Check for arm_only mismatch
                gt_action_dim = self.replay_actions.shape[1]
                model_action_dim = checkpoint["output_dim"]
                if self.is_arm_only and gt_action_dim != model_action_dim:
                    self.get_logger().info(f"Model is arm_only ({model_action_dim} joints) but GT actions have {gt_action_dim} joints. Slicing GT actions.")
                    self.replay_actions = self.replay_actions[:, :model_action_dim]
                
            except Exception as e:
                self.get_logger().fatal(f"Failed to load replay trajectory: {e}")
                raise e
        
        # --- ROS 2 Interfaces ---
        self.policy_arm_joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        self.received_state_order = None 
        self.state_to_policy_indices = None 
        
        self.joint_state_sub = self.create_subscription(JointState, '/franka/joint_states', self.joint_state_callback, 10)
        if self.control_mode == "joint_space":
            self.arm_command_pub = self.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10) # JointTrajectoryController
            # self.arm_command_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10) # MoveIt Servo
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
        self.input_history = []
        self.latest_obs = {} # Stores all raw sensor images
        self.latest_leap_pos = None
        self.latest_vision_features = None

        self.vision_lock = Lock()
        self.sensor_lock = Lock()

        # --- Stateful Tracking for Keypoints ---
        # We now store the last known *position* vector separately from the *full feature* vector.
        # The (x,y,z) or (u,v) part of the feature.
        self.coord_dim = 3 if self.config.get('state', {}).get('use_3d_keypoints', False) else 2

        # --- Get dimensions from the initialized vision_processor ---
        # These are crucial for correctly parsing the feature vector
        self.kp_raw_dim = self.vision_processor.extractor1.output_dim  # 8 in 2d or 10 in 3d
        self.kp_engineered_dim = self.coord_dim # 2 in 2d (u,v), 3 in 3d (x,y,z)
        self.kp_total_dim = self.kp_raw_dim + self.kp_engineered_dim # 10 in 2d, 13 in 3d
        
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

        # P-Control
        self.target_q_arm = None   # Stores the desired accumulated position.
        self.target_q_hand = None  # Stores the desired accumulated position.
        self.smoothed_delta_arm = None
        self.smoothing_alpha = 0.4 # TUNE THIS: 0.0 (max smoothing) to 1.0 (no smoothing)
        self.control_rate_scaling = 1.0 # TUNE THIS: Use > 1.0 (e.g., 1.05) to compensate for JTC tracking lag

        # Control rate
        self.control_rate = self.get_parameter('control_rate_hz').get_parameter_value().double_value
        self.control_rate = self.control_rate  # Slower rollout for testing
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
        for i in range(num_steps +1):
            # Calculate the interpolation factor (alpha) from 0.0 to 1.0
            alpha = i / num_steps
            
            # Linearly interpolate between the open and closed poses
            intermediate_pose = (1 - alpha) * open_hand_pose + alpha * hand_grasp_pose
            
            # Send the command (convert back to list for the function)
            msg = Float64MultiArray()
            msg.data = [float(p) for p in intermediate_pose.tolist()]
            self.hand_command_pub.publish(msg)
            # self.get_logger().info("Hand command sent.")

            # Wait for a short duration before the next step
            time.sleep(grasp_duration / num_steps)
        
        logging.info("Grasp complete.")
        
        logging.info("Starting LEAP position service thread...")
        self.leap_thread = threading.Thread(target=self.leap_service_loop, daemon=True)
        self.leap_thread.start()
        
        logging.info("Starting sensor acquisition thread...")
        self.sensor_thread = threading.Thread(target=self.sensor_acquisition_loop, daemon=True)
        self.sensor_thread.start()

        logging.info("Starting parallel vision processing thread...")
        self.vision_thread = threading.Thread(target=self.vision_processing_loop, daemon=True)
        self.vision_thread.start()
        
        time.sleep(10) # A final short pause after grasping


    def initialize_vision_processor(self):
        data_dirs = [str(paths.RAW_DATA_DIR)] # For depth range calculation
        return VisionProcessor(self.config, self.device, data_dirs)
    
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
        Initializes cameras using specific serial numbers for deterministic order,
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

                if raw_img is None:
                    self.get_logger().info(f"Tactile sensor '{name}' returned a None image. Skipping observation.")
                    return None # Return None for the whole observation if any sensor fails
                obs['tactile_images'][name] = raw_img
                
            return obs
        except Exception as e:
            self.get_logger().info(f"Failed to get live observations: {e}")
            return None
        
    def sensor_acquisition_loop(self):
        """
        This thread's ONLY job is to block on hardware I/O and update
        the self.latest_obs attribute.
        """
        self.get_logger().info("Starting sensor acquisition thread...")
        first_run = True # Flag for initial logging
        while rclpy.ok():
            try:
                local_obs = {}
                # --- Get RealSense Data ---
                frames1 = self.realsense['pipelines']['camera1'].wait_for_frames()
                aligned1 = self.realsense['aligners']['camera1'].process(frames1)
                local_obs['color1'] = np.asanyarray(aligned1.get_color_frame().get_data()).astype(np.uint8)
                local_obs['depth1'] = np.asanyarray(aligned1.get_depth_frame().get_data()).astype(np.uint16)
                
                frames2 = self.realsense['pipelines']['camera2'].wait_for_frames()
                aligned2 = self.realsense['aligners']['camera2'].process(frames2)
                local_obs['color2'] = np.asanyarray(aligned2.get_color_frame().get_data()).astype(np.uint8)
                local_obs['depth2'] = np.asanyarray(aligned2.get_depth_frame().get_data()).astype(np.uint16)
                
                # --- Get Tactile Data ---
                local_obs['tactile_images'] = {}
                tactile_fail = False
                for name, sensor in self.tactile_sensors.items():
                    raw_img = sensor.get_rectify_crop_image()
                    if raw_img is None:
                        self.get_logger().info(f"Tactile sensor '{name}' returned None in acq_thread.", throttle_duration_sec=5)
                        tactile_fail = True
                        break # Don't update this cycle if one sensor fails
                    local_obs['tactile_images'][name] = raw_img
                
                if tactile_fail:
                    continue # Skip this loop iteration

                # --- Publish to Class ---
                # If all data is good, publish it under the lock
                with self.sensor_lock:
                    self.latest_obs = local_obs
                    if first_run:
                        self.get_logger().info("✅ SENSOR ACQUISITION: First successful data package written to self.latest_obs.")
                        first_run = False

            except Exception as e:
                self.get_logger().error(f"FATAL RealSense/Sensor Error in acquisition thread: {e}", throttle_duration_sec=5)
                # You might need to add a small sleep here to prevent rapid failure loops
                time.sleep(0.1)
    
    def initialize_kinematics_solver(self):
        urdf_content = get_urdf_string_from_xacro()
        if not urdf_content: 
            logging.error("Failed to generate URDF, cannot proceed."); return

        self.urdf_temp_file = None
        try:
            # 2b. Save to a temporary file for Pinocchio to load
            # This creates a file with a unique name in the system's temp directory
            with tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.urdf', encoding='utf-8') as f:
                f.write(urdf_content)
                self.urdf_temp_file = Path(f.name)
        
            kinematics_config = self.config.get("kinematics", {})
            self.solver = KinematicsSolver(
                urdf_content=self.urdf_temp_file,
                end_effector_frame_name=kinematics_config.get("ee_frame", "fr3_hand_tcp"),
                tactile_frame_names=kinematics_config.get("tactile_frames", []),
                visualize=False,
            )
            if self.solver is None:
                logging.error("KinematicsSolver failed to initialize model from URDF.")
                return
            logging.info("KinematicsSolver initialized successfully.")

            return self.solver

        except Exception as e:
            logging.error(f"Error initializing KinematicsSolver: {e}")
            return
        # finally:
        #     # Clean up the temporary URDF file
        #     if urdf_temp_file and urdf_temp_file.exists():
        #         urdf_temp_file.unlink()

        
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
        """
        Calls the LEAP position service synchronously.
        This function is called from the custom leap_service_loop thread.
        It must be fully blocking to avoid mixing rclpy executor calls in the custom thread.
        """
        if not self.leap_position_client.wait_for_service(timeout_sec=1.0):
            # Use ROS logger (rclpy.Node method) here because this function is called 
            # from a thread running within the rclpy.Node context. It's safe if 
            # the thread is running while rclpy is active.
            self.get_logger().info("LEAP position service not available.")
            return None

        req = LeapPosition.Request()
        
        # Use the synchronous 'call' method here. This is blocking, but safe within
        # a dedicated Python thread if the primary executor is running elsewhere.
        try:
            # We are blocking here for up to 1 second
            response = self.leap_position_client.call(req) 
            
            if response is not None:
                return np.array(response.position)
            else:
                self.get_logger().info("Failed to get LEAP position: Service call returned None.")
                return None
        except Exception as e:
            self.get_logger().error(f"Exception during synchronous LEAP service call: {e}")
            return None


    def leap_service_loop(self):
        """This thread's only job is to call the LEAP service and update shared state."""
        # Using standard logging or ROS logging is fine here, as it's a dedicated thread 
        # that will shut down cleanly with the main process (daemon=True).
        self.get_logger().info("Starting LEAP position service thread...")
        first_run = True # Flag for initial logging

        while rclpy.ok():
            pos = self.get_current_leap_position()
            
            if pos is not None:
                with self.sensor_lock:
                    self.latest_leap_pos = pos
                if first_run:
                    self.get_logger().info("✅ LEAP SERVICE: Acquired first LEAP position and updated self.latest_leap_pos.")
                    first_run = False
            else:
                self.get_logger().info("LEAP service returned None position.", throttle_duration_sec=5)
            
            # Sleep to prevent spamming the service
            # If the service call itself takes time, the sleep ensures the thread doesn't hog CPU.
            # Using time.sleep(0.02) aims for 50Hz updates.
            time.sleep(0.02)
            
    # --- "Carry Forward" Logic with Staleness Flag ---
    def update_features(self, raw_feats, cam_id):
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
                              
    def vision_processing_loop(self):
        self.get_logger().info("Starting parallel vision processing thread...")
        first_input = True # Flag for initial logging
        while rclpy.ok():
            time_start = time.perf_counter()
            
            # 1. Get the latest raw images (using the sensor lock)
            with self.sensor_lock:
                if 'color1' not in self.latest_obs:
                    time.sleep(0.01) # Wait for data
                    continue
                
                if first_input:
                    self.get_logger().info("✅ VISION PROCESSING: Acquired first input from self.latest_obs.")
                    first_input = False

                color1 = self.latest_obs['color1'].copy()
                depth1 = self.latest_obs['depth1'].copy()
                color2 = self.latest_obs['color2'].copy()
                depth2 = self.latest_obs['depth2'].copy()

            # --- JPEG DEGRADATION WORKAROUND ---
            success1, encoded_image1 = cv2.imencode('.jpg', color1, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if success1:
                color1 = cv2.imdecode(encoded_image1, cv2.IMREAD_COLOR)
            success2, encoded_image2 = cv2.imencode('.jpg', color2, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if success2:
                color2 = cv2.imdecode(encoded_image2, cv2.IMREAD_COLOR)
            
            # 2. Run the SLOW computation
            # self.get_logger().info("--- VISION START: Initiating slow feature extraction. ---")
            self.features_per_cam = self.vision_processor.process_cameras(
                color1, depth1, color2, depth2, frame_id="policy_rollout_latest"
            )
            
            # Initialize a default feature vector in case of failure
            final_features = np.zeros(self.vision_processor.output_dim, dtype=np.float32)

            try:
                # 3. Feature Assembly and Staleness Logic
                # Get the raw "kitchen sink" vectors
                f1_raw = self.features_per_cam.get('cam1', np.zeros(self.single_cam_dim, dtype=np.float32))
                f2_raw = self.features_per_cam.get('cam2', np.zeros(self.single_cam_dim, dtype=np.float32))

                # # Apply complex logic (where the failure might be)
                # f1_final = self.update_features(f1_raw, 'cam1')
                # f2_final = self.update_features(f2_raw, 'cam2')
                f1_final = f1_raw
                f2_final = f2_raw

                # Concatenate the results
                final_features = np.concatenate([f1_final, f2_final])
                
                # self.get_logger().info("Feature assembly successful.")

            except Exception as e:
                # If the assembly fails, log the full traceback and continue the loop with zeros
                self.get_logger().error(f"FATAL EXCEPTION DURING FEATURE ASSEMBLY/STALENESS LOGIC: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                # We use the initialized zero vector for final_features if this fails

            # 4. Store the result under the vision lock
            with self.vision_lock:
                # Ensure the policy will wait again if we hit an exception
                self.latest_vision_features = final_features
            
            # Now, the publish log is guaranteed to fire unless the thread is forcibly killed.
            # self.get_logger().info("--- VISION PUBLISH: Features successfully published and lock released. ---")
            
            time_end = time.perf_counter()
            elapsed = time_end - time_start
            if elapsed > self.sample_period:
                self.get_logger().info(f"Vision processing is slower ({elapsed*1000:.2f} ms) than control period ({self.sample_period*1000:.2f} ms)!", throttle_duration_sec=5)
            # self.get_logger().info(f"Vision thread cycle time: {elapsed*1000:.2f} ms")

    def control_loop(self):
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
        self.MAX_ARM_DELTA = 0.06   # Absolute max delta (rad or m)
        self.ARM_RATE_LIMIT = 0.02  # Max change from the last command
        self.MAX_HAND_DELTA = 0.4   # Absolute max delta for hand joints
        self.HAND_RATE_LIMIT = 0.1 # Max change for hand commands
        # ---`--------------------------------------------------------
        
        self.get_logger().info("✅ All dependencies ready. Starting autonomous control loop.")

        step = 0

        try:
            while rclpy.ok():
                time_start = time.perf_counter()
                
                if self.is_replay_mode:
                    # --- REPLAY MODE ---
                    
                    # 1. Get the current replay step index
                    current_replay_step = self.start_step + step
                    
                    # 2. Check for end of trajectory
                    if current_replay_step >= len(self.replay_actions):
                        self.get_logger().info(f"--- Ground Truth Replay Complete (Reached end of actions) ---")
                        break # Exit the loop
                    
                    # 3. Get the ground truth action (delta)
                    action_pred_np = self.replay_actions[current_replay_step]
                    raw_delta_arm = action_pred_np[:self.num_arm_actions]
                    
                    if not self.is_arm_only:
                        raw_delta_hand = action_pred_np[self.num_arm_actions:]

                    # 4. Initialize Targets on first step (crucial for replay consistency)
                    if step == 0:
                        self.get_logger().info(f"--- REPLAYING TRAJECTORY (Step {step}) ---")
                        self.get_logger().info("Initializing targets from live state for replay...")
                        
                        # Arm initialization (waits for first joint_state_callback outside the loop)
                        self.target_q_arm = self.current_arm_states.copy() 
                        
                        # Hand initialization (must call service)
                        if not self.is_arm_only:
                             leap_pos = self.get_current_leap_position()
                             if leap_pos is not None:
                                self.target_q_hand = leap_pos.copy()
                             else:
                                self.get_logger().fatal("CRITICAL: Failed to get initial LEAP position for replay mode.")
                                break # Exit the control loop on failure
                        
                        self.get_logger().info(f"Applying GT action from t={current_replay_step}: {np.round(raw_delta_arm, 4)}")
                        if not self.is_arm_only:
                            self.get_logger().info(f"Applying GT hand action from t={current_replay_step}: {np.round(raw_delta_hand[:4], 4)}...")

                    # 5. Accumulate Targets (REMOVED: Accumulation must happen later in the publishing block)
                    # self.target_q_arm = self.target_q_arm + raw_delta_arm  <-- BUG FIX: Removed duplicate accumulation here!

                    # 6. Set delta_to_apply_arm (used later in the publishing block)
                    safe_delta_arm = raw_delta_arm # In replay, the raw delta is the "safe" delta

                else:

                    # --- 1. SENSE ---
                    # Note: self.current_arm_states is already set by its own async callback
                    while self.current_arm_states is None:
                        self.get_logger().info("No arm states yet, waiting.", throttle_duration_sec=2)
                        time.sleep(self.sample_period)

                    with self.sensor_lock:
                        # Check if the parallel threads have populated the data
                        while self.latest_leap_pos is None:
                            self.get_logger().info("No LEAP Hand states yet, waiting.", throttle_duration_sec=2)
                            time.sleep(self.sample_period)

                        while 'color1' not in self.latest_obs:
                            self.get_logger().info("Sensor data not ready yet, waiting.", throttle_duration_sec=2)
                            time.sleep(self.sample_period)
                        
                        # Make a local, thread-safe copy of the data for this loop cycle
                        obs = self.latest_obs.copy()
                        leap_positions = self.latest_leap_pos.copy()     

                    if self.solver is None:
                        self.get_logger().fatal("Kinematics solver not initialized. Cannot proceed with control loop.")
                        break


                    self.current_joint_states = np.concatenate([self.current_arm_states, leap_positions])

                    time_sense = time.perf_counter()

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
                    _, kinematic_poses = self.solver.get_all_poses(self.current_arm_states, leap_positions)
                    self.get_logger().info(f"Kinematic Poses: {kinematic_poses}")
                    
                    if self.control_mode == 'task_space':
                        arm_proprio = kinematic_poses['ee'] # 7D EE Pose
                        state_vector_list.append(arm_proprio)
                    else: # joint_space
                        arm_proprio = self.current_arm_states # 7D Arm Joints
                        state_vector_list.append(arm_proprio)            

                    hand_proprio = leap_positions # 16D Hand Joints
                    state_vector_list.append(hand_proprio)
                    current_proprio = np.concatenate([arm_proprio, hand_proprio])

                    self.get_logger().info(f"use 3d tactile: {self.config['state']['use_3d_tactile']}")
                    # 2.3 3D Tactile Poses
                    if self.config['state']['use_3d_tactile']:
                        for frame_name, pose in kinematic_poses.items():
                            self.get_logger().info(f"Pose: {pose}")

                            if frame_name != 'ee' and frame_name in self.config['kinematics']['tactile_frames']:
                                self.get_logger().info(f"Adding 3D tactile pose for frame: {frame_name}")
                                state_vector_list.append(pose)

                    # 2.4 Visual Features (with Carry-Forward Logic)
                    with self.vision_lock:
                        while self.latest_vision_features is None:
                            self.get_logger().info("Vision features not ready. Waiting.")
                            time.sleep(self.sample_period)
                        visual_features = self.latest_vision_features.copy()
                    
                    state_vector_list.append(visual_features)
                    
                    # --- Final State Assembly ---
                    current_state_np = np.concatenate(state_vector_list)
                    if self.use_goal:
                        full_state_np = np.concatenate([current_state_np, self.goal_state.cpu().numpy()])
                    else:
                        full_state_np = current_state_np

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
                    for cam_id, feats in self.features_per_cam.items():
                        if feats is None: continue
                        tube_flag = feats[3]
                        peg_flag = feats[7]
                        visual_log_msg += f"{cam_id} [Tube: {'✅' if tube_flag > 0 else '❌'}, Peg: {'✅' if peg_flag > 0 else '❌'}] | "
                    self.get_logger().info(visual_log_msg)

                    # self.get_logger().info(f"Proprio Shape:          {proprio_data.shape}")
                    # self.get_logger().info(f"Current State Vector Shape: {current_state_np.shape}")
                    # self.get_logger().info(f"Goal State Vector Shape:    {self.goal_state.shape}")
                    
                    time_state = time.perf_counter()

                    # 3. HANDLE HISTORY (Frame Stacking)
                    self.history_buffer.append(full_state_np)
                    if len(self.history_buffer) > self.frame_stack_k:
                        self.history_buffer.pop(0)
                    if len(self.history_buffer) < self.frame_stack_k:
                        self.get_logger().info(f"Filling history buffer... {len(self.history_buffer)}/{self.frame_stack_k}")
                        continue
                    self.get_logger().info(f"------------------------")

                    state_sequence = torch.from_numpy(np.array(self.history_buffer)).float().to(self.device)
                    
                    time_history = time.perf_counter()

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

                    action_pred_np = to_np(action_pred)

                    if current_proprio is not None:
                        self.timestamp_history.append(time_start)
                        self.input_history.append(full_state_np) 
                        self.proprio_history.append(current_proprio)
                        self.prediction_history.append(action_pred.cpu().numpy())

                time_inference = time.perf_counter()

                # 5. PUBLISH COMMAND

                # --- 5.1 Process Arm Command ---
                # NOTE: raw_delta_arm is defined in the IF/ELSE blocks above.
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
                if was_limited:
                    limit_status = "⚠️ CLIPPED/LIMITED"
                    self.get_logger().info(
                        f"ARM CMD {limit_status} | "
                        f"Raw: {np.round(raw_delta_arm, 4)} -> "
                        f"Safe: {np.round(safe_delta_arm, 4)}"
                    )
                else:
                    self.get_logger().info(
                        f"ARM CMD | "
                        f"{np.round(raw_delta_arm, 4)}"
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
                    # --- 1. Get the Safe Delta Command ---
                    # We already have safe_delta_arm from the safety layer.
                    # This is the desired DISPLACEMENT, not velocity.

                    # --- 2. Calculate the Target Position ---

                    # 2a. Apply Time Scaling to compensate for control rate mismatch and JTC tracking lag.
                    # If control rate == policy rate, use a factor like 1.05 to eliminate systematic lag.
                    scaled_delta_arm = safe_delta_arm * self.control_rate_scaling # Restored the rate scaling

                    # 2b. Apply Low-Pass Filter (Exponential Moving Average) for jitter reduction
                    # This smooths the COMMAND only, preserving policy integrity.
                    ALPHA = self.smoothing_alpha 

                    if self.smoothed_delta_arm is None:
                        self.smoothed_delta_arm = scaled_delta_arm
                    else:
                        # EMA: smoothed = (alpha * current_value) + ((1 - alpha) * previous_smoothed)
                        self.smoothed_delta_arm = (ALPHA * scaled_delta_arm) + \
                                                ((1.0 - ALPHA) * self.smoothed_delta_arm)


                    # 2c. Handle initial state where target_q_arm is None (First Execution)
                    if self.target_q_arm is None:
                        # Initialize the internal target to the current measured position.
                        self.target_q_arm = self.current_arm_states.copy() 

                    # 2d. Accumulate the Smoothed Target Delta
                    # Accumulate the delta onto our *maintained target state*.
                    # This is the ONLY place accumulation should happen for joint_space control.
                    self.target_q_arm = self.target_q_arm + self.smoothed_delta_arm

                    # --- 3. Publish as a Position Command ---
                    traj_msg = JointTrajectory()
                    traj_msg.joint_names = self.policy_arm_joint_names

                    point = JointTrajectoryPoint()
                    # Command the robot to go to this new, accumulated target position.
                    point.positions = self.target_q_arm.tolist()

                    duration_ns = int(self.sample_period * 1e9)
                    point.time_from_start = Duration(sec=0, nanosec=duration_ns)

                    traj_msg.points.append(point)
                    self.arm_command_pub.publish(traj_msg)

                # --- 5.2 Process Hand Command (with same safety logic) ---
                if not self.is_arm_only:
                    
                    if not self.is_replay_mode:
                        # --- Policy Mode: Safety Check & Delta Selection ---
                        raw_delta_hand = action_pred_np[self.num_arm_actions:]
                        
                        # Apply clipping and rate limiting
                        clipped_delta_hand = np.clip(raw_delta_hand, -self.MAX_HAND_DELTA, self.MAX_HAND_DELTA)
                        change_hand = clipped_delta_hand - self.previous_hand_command
                        limited_change_hand = np.clip(change_hand, -self.HAND_RATE_LIMIT, self.HAND_RATE_LIMIT)
                        safe_delta_hand = self.previous_hand_command + limited_change_hand
                        
                        # Update for the next cycle
                        self.previous_hand_command = safe_delta_hand
                        delta_to_apply_hand = safe_delta_hand

                        # --- Diagnostic Logging for Hand ---
                        was_limited_hand = not np.allclose(raw_delta_hand, safe_delta_hand, atol=1e-6)
                        limit_status_hand = "⚠️ CLIPPED/LIMITED" if was_limited_hand else "✅ OK"
                        self.get_logger().info(
                            f"HAND CMD | Status: {limit_status_hand} | "
                            f"Raw: {np.round(raw_delta_hand[:4], 4)}... -> " # Log first 4 joints for brevity
                            f"Safe: {np.round(safe_delta_hand[:4], 4)}..."
                        )

                        # Handle initial state where target_q_hand is None (First Execution)
                        if self.target_q_hand is None:
                            if leap_positions is not None:
                                self.target_q_hand = leap_positions.copy()
                            else:
                                # This should have been caught in SENSE phase, but as fallback:
                                self.get_logger().info("Could not initialize hand target (policy mode). Skipping command.")
                                time.sleep(self.sample_period)
                                continue
                    
                    else:
                        # --- Replay Mode: Delta was already calculated and target was initialized on step 0 ---
                        delta_to_apply_hand = raw_delta_hand

                    # --- Hand Target Accumulation & Publishing ---
                    
                    # Accumulate the delta onto the maintained target state
                    self.target_q_hand = self.target_q_hand + delta_to_apply_hand

                    hand_msg = Float64MultiArray()
                    hand_msg.data = [float(p) for p in self.target_q_hand]
                    self.hand_command_pub.publish(hand_msg)
                
                step +=1
                time_publish = time.perf_counter()

                total_time = time_publish - time_start

                if total_time > self.sample_period:
                    self.get_logger().warning(f"Loop Overrun: {total_time*1000:.1f}ms")
                    if not self.is_replay_mode:

                        self.get_logger().info(
                            f"  Sense: {(time_sense - time_start)*1000:.1f}ms | "
                            f"Features: {(time_state - time_sense)*1000:.1f}ms | "
                            f"History: {(time_history - time_state)*1000:.1f}ms | "
                            f"Inference: {(time_inference - time_history)*1000:.1f}ms | "
                            f"Publish: {(time_publish - time_inference)*1000:.1f}ms"
                        )
                else:
                    self.get_logger().info(f"Loop time: {total_time*1000:.1f}ms")
                    if not self.is_replay_mode:
                        self.get_logger().info(
                            f"  Sense: {(time_sense - time_start)*1000:.1f}ms | "
                            f"Features: {(time_state - time_sense)*1000:.1f}ms | "
                            f"History: {(time_history - time_state)*1000:.1f}ms | "
                            f"Inference: {(time_inference - time_history)*1000:.1f}ms | "
                            f"Publish: {(time_publish - time_inference)*1000:.1f}ms"
                        )



                time_sleep = max(0, self.sample_period - (time_publish - time_start))
                time.sleep(time_sleep)

        except KeyboardInterrupt:
            logging.info("Rollout interrupted by user.")

        finally:
            self.save_trajectory_data()
            self.publish_smooth_stop()


    def save_trajectory_data(self):
        """Saves the collected trajectory data to a compressed numpy file."""
        if not self.proprio_history:
            self.get_logger().info("No trajectory data to save.")
            return

        self.get_logger().info(f"💾 Saving trajectory data to rollout_data.npz with inputs...")
        
        # Use np.stack to convert list of arrays into a single large array
        np.savez_compressed(
            str(paths.MODELS_DIR/f"rollout/rollout_{self.model_type}_{self.control_mode}.npz"),
            timestamps=np.array(self.timestamp_history),
            inputs=np.stack(self.input_history), 
            proprioception=np.stack(self.proprio_history),
            predictions=np.stack(self.prediction_history)
        )
        self.get_logger().info("✅ Trajectory data saved successfully.")

    def publish_smooth_stop(self):
        """
        Sends a final trajectory command with a long duration (0.5s) 
        to allow the robot to decelerate smoothly to its last target.
        This should be called on node shutdown or loop exit.
        """
        if self.target_q_arm is None:
            self.get_logger().info("Cannot send smooth stop; target_q_arm was never initialized.")
            return

        self.get_logger().info("Sending final smooth stop command...")
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.policy_arm_joint_names
        num_joints = len(self.policy_arm_joint_names)

        stop_point = JointTrajectoryPoint()
        
        # 1. Use the last known target position
        stop_point.positions = self.target_q_arm.tolist()

        # 2. Command zero velocity
        stop_point.velocities = [0.0] * num_joints

        # 3. CRITICAL: Set a LONG duration for smooth deceleration
        stop_point.time_from_start = Duration(sec=2, nanosec=0) 

        traj_msg.points.append(stop_point)
        self.arm_command_pub.publish(traj_msg)
        
        # IMPORTANT: Pause here. We must keep the node alive long enough
        # for the command to be published AND for the JTC to execute 
        # a significant portion of the 0.5s trajectory.
        # Sleeping for slightly longer than the command duration is safest.
        self.get_logger().info("Waiting 0.7s for stop command to execute...")
        time.sleep(0.7) 
        self.get_logger().info("Smooth stop command sent. Node will now exit.")

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