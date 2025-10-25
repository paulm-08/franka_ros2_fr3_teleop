"""
dataset_builder.py (Revised)

- Processes raw multimodal data (tactile, visual, proprio) from demonstrations.
- Uses the revised tactile_features script to get rich 8D features.
- Correctly handles multiple clips and demonstrations as separate trajectories.
- Saves the final dataset as a list of trajectories in a .pkl file to preserve
  boundaries, which is critical for correct data splitting.

- Now supports swappable vision modules (ResNet Embedder vs. YOLO Keypoint Extractor)
  driven by a configuration dictionary.
- Implements a stateful "carry forward" strategy for YOLO keypoints to robustly
  handle temporary occlusions and failed detections.
- Correctly processes and formats visual features from two separate cameras.

- Reads main_config.yaml to build a dataset based on the specified
  control_mode ('task_space' or 'joint_space') and state representation.
- Uses KinematicsSolver to generate 3D poses for the end-effector and sensors.
- Generates 6D task-space actions (delta_pose) for the arm.

"""
import os
import glob
import cv2
import numpy as np
import logging
import argparse
import pickle
from pathlib import Path
import json
import torch
import yaml
import pinocchio as pin
import inquirer
import tempfile

from model_pipeline.visual_embedder import VisualEmbedder
from model_pipeline.keypoint_extractor import KeypointExtractor
from model_pipeline.tactile_features import process_tactile_image, TACTILE_FEATURE_DIM
from model_pipeline.utils import load_frame_paths, load_actions, get_cfg_path, init_sensor
from model_pipeline import paths
from model_pipeline.kinematics import KinematicsSolver, get_urdf_string_from_xacro


# It's good practice to define sensor names for consistent ordering
SENSOR_ORDER = ["rindex", "rmiddle", "rthumb"]

# --- Logger Setup ---
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()]
)

class VisionProcessor:
    """A wrapper class to handle different vision modules (ResNet/YOLO) seamlessly."""
    def __init__(self, config, device, data_dirs):
        state_config = config.get("state", {})
        self.use_keypoint_extractor = state_config.get("use_keypoint_extractor", False)
        self.use_3d_keypoints = state_config.get("use_3d_keypoints", False)
        self.use_resnet_embeddings = state_config.get("use_resnet_embeddings", False)

        vision_config = config.get("vision", {})

        if not (self.use_keypoint_extractor or self.use_resnet_embeddings):
            raise ValueError("At least one vision module (Keypoint Extractor or ResNet Embedder) must be enabled.")
        
        self.keypoint_dim_per_cam = 0
        self.embedder_rgb_dim = 0
        self.embedder_depth_dim = 0

        if self.use_keypoint_extractor:
            logging.info("Initializing VisionProcessor with YOLO Keypoint Extractor...")
        
            # 1. Initialize the KeypointExtractor (for 2D or 3D features)
            self.extractor1 = KeypointExtractor(
                model_path=str(paths.WORKSPACE_ROOT / vision_config["yolo_model_path"]),
                use_3d=self.use_3d_keypoints,
                confidence_threshold=vision_config.get("confidence_threshold", 0.1),
                intrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("intrinsics_path_cam1")),
                extrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("extrinsics_path_cam1")),
                device=device
            )

            self.extractor2 = KeypointExtractor(
                model_path=str(paths.WORKSPACE_ROOT / vision_config["yolo_model_path"]),
                use_3d=self.use_3d_keypoints,
                confidence_threshold=vision_config.get("confidence_threshold", 0.1),
                intrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("intrinsics_path_cam2")),
                extrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("extrinsics_path_cam2")),
                device=device
            )

            keypoint_dim = 3 if self.use_3d_keypoints else 2 # x,y,z or u,v for the relative vector
            self.keypoint_dim_per_cam = self.extractor1.output_dim + keypoint_dim  # (8 + 2 = 10) or (10 + 3 = 13) if 3D
            logging.info(f"KeypointExtractor initialized. Single-camera dim: {self.keypoint_dim_per_cam} (including relative vector).")

        if self.use_resnet_embeddings:
            logging.info("Initializing VisionProcessor with ResNet Embedder...")

            # 2. Initialize the ResNet Embedder (for RGB and Depth context)
            global_depth_range = config.get("global_depth_range")
            if not global_depth_range:
                logging.warning("Global depth range not found in config, computing it now. This can be slow.")
                global_depth_range = compute_global_depth_range(data_dirs)

            # ... (load depth range)
            self.embedder = VisualEmbedder(
                backbone=config.get("backbone", "resnet18"), device=device,
                out_dim={'rgb': config.get("visual_dim", 256), 'depth': config.get("depth_dim", 128)},
                global_depth_range=global_depth_range
            )
            self.embedder_rgb_dim = self.embedder.out_dim['rgb']
            self.embedder_depth_dim = self.embedder.out_dim['depth']
            logging.info(f"VisualEmbedder initialized. RGB dim: {self.embedder_rgb_dim}, Depth dim: {self.embedder_depth_dim}.")

        # 3. Define the final, combined feature dimension
        self.single_cam_dim = self.keypoint_dim_per_cam + self.embedder_rgb_dim + self.embedder_depth_dim
        self.output_dim = self.single_cam_dim * 2
        
        logging.info(f"Vision module initialized. Single-camera dim: {self.single_cam_dim}, Total visual dim: {self.output_dim}")
    
    def process_cameras(self, color1, depth1, color2, depth2):
        """Processes images from both cameras based on the selected vision mode."""
        keypoint_feats1 = []
        keypoint_feats2 = []
        embedding_feats1 = []
        embedding_feats2 = []

        if self.use_keypoint_extractor:
            def get_engineered_features(extractor, color, depth):
                raw_kps = extractor.extract_scene_features(color, depth)
                if raw_kps is None: return np.zeros(self.single_cam_dim, dtype=np.float32)

                # Feature order: [tube(N), peg(N)]
                tube_kp = raw_kps[0 : extractor.feature_dim_per_object]
                peg_kp = raw_kps[extractor.feature_dim_per_object : ]
                
                # Use 3D or 2D coordinates for relative vector
                coord_dim = 3 if extractor.use_3d else 2
                
                if tube_kp[coord_dim+1] > 0 and peg_kp[coord_dim+1] > 0: # Check detection flags
                    relative_vec = tube_kp[0:coord_dim] - peg_kp[0:coord_dim]
                else:
                    relative_vec = np.zeros(coord_dim, dtype=np.float32)
                
                # Final vector per camera: [raw_kps(8D or 10D), relative_vec(2D or 3D)]
                return np.concatenate([raw_kps, relative_vec])
            
            keypoint_feats1 = get_engineered_features(self.extractor1, color1, depth1)
            keypoint_feats2 = get_engineered_features(self.extractor2, color2, depth2)

        if self.use_resnet_embeddings: # ResNet Embedder
            def extract(c_img, d_img):
                if c_img is None: return np.zeros(self.single_cam_dim, dtype=np.float32)
                rgb = self.embedder.embed_rgb(c_img)
                depth = self.embedder.embed_depth(d_img)
                if rgb is None or depth is None: return np.zeros(self.single_cam_dim, dtype=np.float32)
                return np.concatenate([rgb, depth])
            embedding_feats1 = extract(color1, depth1)
            embedding_feats2 = extract(color2, depth2)

        # --- Combine features based on enabled modules ---
        feats1 = np.concatenate([
            keypoint_feats1,
            embedding_feats1
        ])
        feats2 = np.concatenate([
            keypoint_feats2,
            embedding_feats2
        ])
        return {'cam1': feats1, 'cam2': feats2}
    
def calculate_6d_pose_delta(pose_start: np.ndarray, pose_end: np.ndarray) -> np.ndarray:
    """
    Calculates the 6D (twist) delta from a start pose to an end pose.
    Poses are [x, y, z, qx, qy, qz, qw].
    Returns a 6D vector [vx, vy, vz, wx, wy, wz].
    """
    # Convert [x,y,z,qx,qy,qz,qw] to Pinocchio's SE3 object
    T_start = pin.XYZQUATToSE3(pose_start)
    T_end = pin.XYZQUATToSE3(pose_end)
    
    # Calculate the transformation from start to end
    T_delta = T_start.inverse() * T_end
    
    # Convert the SE3 transformation to a 6D twist/motion vector
    # This is the "action" that moves from T_start to T_end
    return pin.log(T_delta).vector
    
def find_demo_dirs(root_search_path):
    """
    Recursively finds all valid demonstration directories.
    """
    logging.info(f"Searching for demonstration directories in: {root_search_path}...")
    found_demos = []
    for dirpath, _, _ in os.walk(root_search_path):
        if glob.glob(os.path.join(dirpath, 'frame_*')):
            relative_path = Path(dirpath).relative_to(paths.WORKSPACE_ROOT)
            found_demos.append(str(relative_path))
    logging.info(f"Found {len(found_demos)} potential demonstration directories.")
    return sorted(found_demos)

def find_config_files(root_search_path):
    """
    Finds all .yaml configuration files in the specified directory.
    """
    logging.info(f"Searching for configuration files in: {root_search_path}...")
    found_configs = [p.relative_to(paths.WORKSPACE_ROOT) for p in root_search_path.glob("*.yaml")]
    logging.info(f"Found {len(found_configs)} config files.")
    return [str(p) for p in found_configs]

def compute_global_depth_range(data_dirs, percentile=99.5):
    """
    (Memory-Efficient Version)
    Iterates through all depth images to find a robust global min and max value
    using a histogram to avoid loading all data into memory.
    """
    logging.info("Calculating global depth range (memory-efficient)...")

    all_depth_paths = []
    for data_dir in data_dirs:
        all_depth_paths.extend(glob.glob(os.path.join(data_dir, "frame_*", "depth_image*.png")))

    if not all_depth_paths:
        logging.warning("No depth images found.")
        return 0, 1000

    # Initialize statistics
    min_val = float('inf')
    # Create a histogram to store the distribution of depth values.
    # The range is 0-65536 for 16-bit images.
    num_bins = 65536
    hist_range = (0, num_bins)
    global_hist = np.zeros(num_bins, dtype=np.int64) # Use 64-bit int to prevent overflow
    total_pixel_count = 0

    # --- Streaming Pass: Process one image at a time ---
    for path in all_depth_paths:
        depth_img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if depth_img is not None:
            # Filter out invalid zero-value pixels
            valid_pixels = depth_img[depth_img > 0]
            if valid_pixels.size > 0:
                # Update the running minimum
                min_val = min(min_val, np.min(valid_pixels))

                # Update the global histogram with this image's data
                local_hist, _ = np.histogram(valid_pixels, bins=num_bins, range=hist_range)
                global_hist += local_hist
                total_pixel_count += valid_pixels.size

    if total_pixel_count == 0:
        logging.warning("No valid depth pixels found. Using default range [0, 1000].")
        return 0, 1000

    # --- Calculate Percentile from the Final Histogram ---
    # Find the pixel count that corresponds to the desired percentile
    percentile_threshold = total_pixel_count * (percentile / 100.0)

    # Find the bin where the cumulative sum of pixels exceeds the threshold
    cumulative_hist = np.cumsum(global_hist)
    # np.searchsorted finds the index where the threshold would be inserted to maintain order
    max_val = np.searchsorted(cumulative_hist, percentile_threshold)

    logging.info(f"✅ Robust global depth range found: min={min_val:.0f}, max={max_val:.0f} (at {percentile}th percentile)")
    return min_val, max_val

def process_single_trajectory(frame_dirs, ref_frame_dir, vision_processor, solver, config):
    """
    Processes a single continuous trajectory (a clip or a full demonstration).
    This function is now stateful to handle carrying forward keypoint detections across frames.

    Returns:
        A dictionary containing the processed data for this trajectory, or None if invalid.
    """
    logging.info(f"Processing trajectory of length {len(frame_dirs)} with reference {os.path.basename(ref_frame_dir)}")

    # --- Load actions for this specific trajectory ---
    actions = np.array(load_actions(frame_dirs), dtype=np.float32)
    if len(actions) < 2:
        logging.warning("Trajectory too short (< 2 frames), skipping.")
        return None

    # --- Initialize sensors with the reference frame for this trajectory ---
    ref_tactile = {
        os.path.basename(p): cv2.imread(p, cv2.IMREAD_UNCHANGED)
        for p in glob.glob(os.path.join(ref_frame_dir, "*raw_image.jpg"))
    }

    use_height_map = config.get("state", {}).get("use_height_map", True)

    sensors = {}
    if use_height_map:
        for sensor_name in SENSOR_ORDER:
            cfg_path = get_cfg_path(sensor_name.replace("r", ""))
            ref_img = ref_tactile.get(f"{sensor_name}_raw_image.jpg")
            if ref_img is not None:
                sensors[sensor_name] = init_sensor(cfg_path=cfg_path, calibrated=True, ref=ref_img)
            else:
                logging.warning(f"Reference image for {sensor_name} not found.")
                sensors[sensor_name] = None

    # --- Stateful Tracking for Keypoints ---
    # We now store the last known *position* vector separately from the *full feature* vector.
    # The (x,y,z) or (u,v) part of the feature.
    coord_dim = 3 if config.get('state', {}).get('use_3d_keypoints', False) else 2

    # --- Get dimensions from the initialized vision_processor ---
    # These are crucial for correctly parsing the feature vector
    kp_raw_dim = vision_processor.extractor1.output_dim  # e.g., 10 (5 per object)
    kp_engineered_dim = coord_dim
    kp_total_dim = kp_raw_dim + kp_engineered_dim # e.g., 13
    
    single_cam_dim = vision_processor.single_cam_dim # e.g., 61

    
    # Store the last known [x,y,z] or [u,v]
    last_known_coords = {
        'cam1_tube': np.zeros(coord_dim, dtype=np.float32),
        'cam1_peg':  np.zeros(coord_dim, dtype=np.float32),
        'cam2_tube': np.zeros(coord_dim, dtype=np.float32),
        'cam2_peg':  np.zeros(coord_dim, dtype=np.float32)
    }

    # --- Per-frame feature extraction ---
    tactile_list, visual_list = [], []
    kinematic_pose_list, hand_joint_list = [], []

    for i, frame_dir in enumerate(frame_dirs):
        # --- Tactile Features (with guaranteed order) ---
        frame_feats = []
        # Sort glob results to ensure consistent feature order
        tactile_img_paths = sorted(glob.glob(os.path.join(frame_dir, "*raw_image.jpg")))

        # Create a map for quick path lookup
        path_map = {os.path.basename(p): p for p in tactile_img_paths}

        for sensor_name in SENSOR_ORDER:
            fname = f"{sensor_name}_raw_image.jpg"
            img_path = path_map.get(fname)
            
            if img_path:
                img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
                ref = ref_tactile.get(fname)
                sensor = sensors.get(sensor_name) if use_height_map else None

                # Use the new feature extractor (visualization is off for speed)
                feature_vector, _ = process_tactile_image(
                    img, ref_img=ref, use_height_map=use_height_map, sensor=sensor, generate_visualization=False
                )
                frame_feats.extend(feature_vector)
            else:
                # If a sensor image is missing for this frame, append zeros
                frame_feats.extend(np.zeros(TACTILE_FEATURE_DIM, dtype=np.float32))
        
        tactile_list.append(frame_feats)

        # --- Visual Embeddings (RGB + Depth) ---
        color1_path = os.path.join(frame_dir, "color_image1.jpg")
        depth1_path = os.path.join(frame_dir, "depth_image1.png") # Or .tiff, .npy, etc.
        color2_path = os.path.join(frame_dir, "color_image2.jpg")
        depth2_path = os.path.join(frame_dir, "depth_image2.png")

        # Load images, using IMREAD_UNCHANGED for depth to preserve bit depth (e.g., 16-bit)
        color1 = cv2.imread(color1_path)
        depth1 = cv2.imread(depth1_path, cv2.IMREAD_UNCHANGED)
        color2 = cv2.imread(color2_path)
        depth2 = cv2.imread(depth2_path, cv2.IMREAD_UNCHANGED)
        
        features_per_cam = vision_processor.process_cameras(color1, depth1, color2, depth2)

        # --- "Carry Forward" Logic with Staleness Flag ---
        def update_features(raw_feats, cam_id):
            """
            Parses a 'kitchen sink' vector, applies staleness logic to the
            keypoint part, and preserves the embedding part.
            """
            if raw_feats is None: # Handle case where a camera failed
                return np.zeros(single_cam_dim, dtype=np.float32)

            # --- 1. Split the "kitchen sink" vector ---
            kps_vec = raw_feats[:kp_total_dim]
            embs_vec = raw_feats[kp_total_dim:]
            
            # Deconstruct the keypoint vector
            # [tube(5), peg(5), rel(3)]
            tube_feats = kps_vec[0:coord_dim+2]
            peg_feats = kps_vec[coord_dim+2:2*coord_dim+4]
            
            tube_key, peg_key = f"{cam_id}_tube", f"{cam_id}_peg"

            # --- 2. Apply Staleness Logic ---
            # Check TUBE detection (flag is at index 4)
            if tube_feats[coord_dim+1] > 0: # if flag is 1
                last_known_coords[tube_key] = tube_feats[0:coord_dim] # Store new 3D pos
            else:
                tube_feats[0:coord_dim] = last_known_coords[tube_key] # Use stale position
                tube_feats[coord_dim:coord_dim+2] = 0.0 # Set conf and flag to 0

            # Check PEG detection (flag is at index 4 of its vector)
            if peg_feats[coord_dim+1] > 0: # if flag is 1
                last_known_coords[peg_key] = peg_feats[0:coord_dim]
            else:
                peg_feats[0:coord_dim] = last_known_coords[peg_key]
                peg_feats[coord_dim:coord_dim+2] = 0.0
            
            # --- 3. Re-engineer the relative vector ---
            # This is crucial: we recalculate the relative vector based on the
            # (potentially stale) coordinates we are passing to the policy.
            rel_vec = tube_feats[0:coord_dim] - peg_feats[0:coord_dim]
            
            # Reconstruct the final keypoint vector
            final_kps_vec = np.concatenate([tube_feats, peg_feats, rel_vec])
            
            # --- 4. Recombine with embeddings ---
            # Return the full, corrected "kitchen sink" vector
            return np.concatenate([final_kps_vec, embs_vec])

        # Get the raw "kitchen sink" vectors
        f1_raw = features_per_cam.get('cam1', np.zeros(single_cam_dim, dtype=np.float32))
        f2_raw = features_per_cam.get('cam2', np.zeros(single_cam_dim, dtype=np.float32))

        # Apply the staleness logic
        f1_final = update_features(f1_raw, 'cam1')
        f2_final = update_features(f2_raw, 'cam2')
        
        visual_list.append(np.concatenate([f1_final, f2_final]))
        # --- Kinematics (End-Effector Pose and Hand Joints) ---
        arm_joints = actions[i, :7]
        hand_joints = actions[i, 7:23]
        
        _, kinematic_poses = solver.get_all_poses(arm_joints, hand_joints)
        kinematic_pose_list.append(kinematic_poses)
        hand_joint_list.append(hand_joints)

    # --- Convert lists to numpy arrays ---
    tactile_feats = np.array(tactile_list, dtype=np.float32)
    visual_feats = np.stack(visual_list, axis=0)
    kinematic_poses_array = np.array(kinematic_pose_list) # Array of dicts
    hand_joints_array = np.array(hand_joint_list, dtype=np.float32)

    # --- Build State, Goal, and Action vectors based on config ---
    state_list, goal_list = [], []

    # 1. Tactile Features (always included)
    state_list.append(tactile_feats)
    goal_list.append(tactile_feats[-1])

    # 2. Proprioceptive Features
    if config['control_mode'] == 'task_space':
        # Extract 7D EE pose [x,y,z,qx,qy,qz,qw] from the array of dicts
        ee_poses = np.array([p['ee'] for p in kinematic_poses_array])
        state_list.append(ee_poses)
        goal_list.append(ee_poses[-1])
    else: # joint_space
        state_list.append(actions[:, :7]) # 7D Arm Joints
        goal_list.append(actions[-1, :7])
    
    state_list.append(hand_joints_array) # Always include 16D Hand Joints
    goal_list.append(hand_joints_array[-1])

    # 3. 3D Tactile Poses
    if config['state']['use_3d_tactile']:
        for frame_name in config['kinematics']['tactile_frames']:
            poses = np.array([p[frame_name] for p in kinematic_poses_array])
            state_list.append(poses)
            goal_list.append(poses[-1])

    # 4. Visual Features
    # The visual_feats is already the full [keypoints, embeddings] vector
    if config['state']['use_keypoint_extractor'] or config['state']['use_resnet_embeddings']:
        state_list.append(visual_feats)
        goal_list.append(visual_feats[-1])
        
    # --- Finalize State and Goal ---
    s_t_full = np.concatenate(state_list, axis=1)
    goal_state = np.concatenate(goal_list)
    
    # Create slices for s_t and g_t
    s_t = s_t_full[:-1]
    g_t = np.tile(goal_state, (s_t.shape[0], 1))

    # --- Build Action Vector (a_t) ---
    if config['control_mode'] == 'task_space':
        ee_poses = np.array([p['ee'] for p in kinematic_poses_array])
        delta_arm_action = []
        for i in range(len(ee_poses) - 1):
            delta = calculate_6d_pose_delta(ee_poses[i], ee_poses[i+1])
            delta_arm_action.append(delta)
        delta_arm = np.array(delta_arm_action)
    else: # joint_space
        delta_arm = actions[1:, :7] - actions[:-1, :7]
        
    delta_hand = actions[1:, 7:23] - actions[:-1, 7:23]
    a_t = np.concatenate([delta_arm, delta_hand], axis=1)

    # --- Sanity Checks ---
    assert s_t.shape[0] == a_t.shape[0] == g_t.shape[0], "State, Action, and Goal have different lengths!"
    
    return {
        "state_t": s_t.astype(np.float32),
        "goal_t": g_t.astype(np.float32),
        "action_t": a_t.astype(np.float32)
    }

# === NEW: Config-Aware Dataset Summary ===
def print_stats(name, data):
    """Helper function to print statistics for a data block."""
    logging.info(f"\n--- {name} (Shape: {data.shape}) ---")
    if np.isnan(data).any() or np.isinf(data).any():
        logging.error(f"  ❌ Found NaN or Inf values in {name} data!")
    else:
        logging.info(f"  ✅ No NaN or Inf values detected.")
    
    min_val = np.min(data)
    max_val = np.max(data)
    mean_val = np.mean(data)
    std_val = np.std(data)
    
    logging.info(f"  Min: {min_val:.4f}, Max: {max_val:.4f}, Mean: {mean_val:.4f}, Std: {std_val:.4f}")

def generate_dataset_summary(all_trajectories, config, vision_processor):
    """
    Calculates and prints a config-aware statistical summary of the new dataset structure.
    """
    if not all_trajectories:
        logging.warning("Cannot generate summary for an empty dataset.")
        return

    logging.info("\n" + "="*60)
    logging.info("          🤖 DATASET STATISTICAL SUMMARY 🤖")
    logging.info("="*60)

    # --- Aggregate all data ---
    all_states = np.concatenate([t['state_t'] for t in all_trajectories], axis=0)
    all_goals = np.concatenate([t['goal_t'] for t in all_trajectories], axis=0)
    all_actions = np.concatenate([t['action_t'] for t in all_trajectories], axis=0)
    traj_lengths = [len(t['state_t']) for t in all_trajectories]
    total_samples = all_states.shape[0]

    # --- Overall Summary ---
    logging.info(f"Number of trajectories: {len(all_trajectories)}")
    logging.info(f"Total number of samples: {total_samples}")
    logging.info(f"Trajectory Lengths | Min: {min(traj_lengths)}, Max: {max(traj_lengths)}, Avg: {np.mean(traj_lengths):.1f}")
    
    logging.info(f"\n--- State (s_t) Breakdown (Total Dim: {all_states.shape[1]}) ---")
    current_idx = 0

    # 1. Tactile Features
    tactile_dim = len(SENSOR_ORDER) * TACTILE_FEATURE_DIM
    print_stats("  - Tactile Features", all_states[:, current_idx:current_idx+tactile_dim])
    current_idx += tactile_dim

    # 2. Proprioceptive Features (Arm)
    control_mode = config.get('control_mode', 'joint_space')
    arm_proprio_dim = 7 # 7D pose [x,y,z,qx,qy,qz,qw] or 7D joints
    if control_mode == 'task_space':
        print_stats("  - Arm Proprio (Task Space Pose)", all_states[:, current_idx:current_idx+arm_proprio_dim])
    else:
        print_stats("  - Arm Proprio (Joint Space)", all_states[:, current_idx:current_idx+arm_proprio_dim])
    current_idx += arm_proprio_dim

    # 3. Proprioceptive Features (Hand)
    hand_proprio_dim = 16 # 16D hand joints
    print_stats("  - Hand Proprio (Joint Space)", all_states[:, current_idx:current_idx+hand_proprio_dim])
    current_idx += hand_proprio_dim

    # 4. 3D Tactile Poses
    if config.get('state', {}).get('use_3d_tactile'):
        tactile_frames = config.get('kinematics', {}).get('tactile_frames', [])
        tactile_pose_dim = len(tactile_frames) * 7 # 7D pose per frame
        print_stats(f"  - 3D Tactile Poses ({len(tactile_frames)} frames)", all_states[:, current_idx:current_idx+tactile_pose_dim])
        current_idx += tactile_pose_dim

    # 5. Visual Features
    vis_dim = 0
    if config.get('state', {}).get('use_keypoint_extractor') or config.get('state', {}).get('use_resnet_embeddings'):
        vis_dim = vision_processor.output_dim
        print_stats("  - Visual Features", all_states[:, current_idx:current_idx+vis_dim])
        current_idx += vis_dim
    
    if current_idx != all_states.shape[1]:
        logging.error(f"  ❌ STATE DIMENSION MISMATCH! Config implies {current_idx}D, but data has {all_states.shape[1]}D.")

    # --- Goal Breakdown (Same structure as State) ---
    # We can just do a high-level summary for the goal
    print_stats("Full Goal (g_t)", all_goals)

    # --- Action Breakdown ---
    logging.info(f"\n--- Action (a_t) Breakdown (Total Dim: {all_actions.shape[1]}) ---")
    current_idx = 0
    
    if control_mode == 'task_space':
        arm_action_dim = 6 # 6D twist
        print_stats("  - Arm Action (Task Space Delta)", all_actions[:, current_idx:current_idx+arm_action_dim])
        current_idx += arm_action_dim
    else: # joint_space
        arm_action_dim = 7 # 7D joint delta
        print_stats("  - Arm Action (Joint Space Delta)", all_actions[:, current_idx:current_idx+arm_action_dim])
        current_idx += arm_action_dim
    
    hand_action_dim = 16
    print_stats("  - Hand Action (Joint Space Delta)", all_actions[:, current_idx:current_idx+hand_action_dim])
    current_idx += hand_action_dim
    
    if current_idx != all_actions.shape[1]:
        logging.error(f"  ❌ ACTION DIMENSION MISMATCH! Config implies {current_idx}D, but data has {all_actions.shape[1]}D.")
        
    logging.info("="*60)

def log_detailed_tactile_analysis(all_trajectories):
    """
    Performs a deep dive into the generated tactile data to check for integrity issues,
    specifically focusing on the contact flags and force values.
    """
    if not all_trajectories:
        return

    logging.info("\n" + "="*60)
    logging.info("        🔬 DETAILED TACTILE SANITY CHECK 🔬")
    logging.info("="*60)
    
    all_tactile = np.concatenate([traj['tactile_t'] for traj in all_trajectories], axis=0)
    total_samples = all_tactile.shape[0]

    for i, sensor_name in enumerate(SENSOR_ORDER):
        # Define the column indices for this sensor's features
        start_idx = i * TACTILE_FEATURE_DIM
        force_idx = start_idx + 6
        flag_idx = start_idx + 7
        
        # Extract this sensor's data across all samples
        sensor_flags = all_tactile[:, flag_idx]
        sensor_forces = all_tactile[:, force_idx]
        
        # --- Perform the Checks ---
        contact_frames = np.sum(sensor_flags == 1.0)
        no_contact_frames = np.sum(sensor_flags == 0.0)
        
        logging.info(f"\n--- Analysis for Sensor: '{sensor_name}' ---")
        logging.info(f"  Total Samples: {total_samples}")
        logging.info(f"  Frames flagged as NO CONTACT (flag=0): {no_contact_frames}")
        logging.info(f"  Frames flagged as CONTACT (flag=1): {contact_frames}")

        # This is the most critical check
        if (contact_frames + no_contact_frames) != total_samples:
            unknown_flag_frames = total_samples - (contact_frames + no_contact_frames)
            logging.error(f"  ❌ [FATAL ERROR] Found {unknown_flag_frames} frames with an invalid contact flag (not 0 or 1)!")
        else:
            logging.info("  ✅ Contact flags are consistent (all are 0.0 or 1.0).")

        # Check for light touches among the contact frames
        if contact_frames > 0:
            contact_mask = sensor_flags == 1.0
            zero_force_on_contact = np.sum(sensor_forces[contact_mask] <= 1e-6)
            logging.info(f"  Of the {contact_frames} CONTACT frames, {zero_force_on_contact} have a near-zero force ('light touch').")
    
    logging.info("="*60)

def build_dataset(data_dirs, out_file, config):
    """
    Builds a dataset from multiple raw data directories.

    Args:
        data_dirs (list): List of paths to the raw demonstration directories.
        out_file (str): Path to save the output .pkl file.
        use_height_map (bool): Whether to use the height map for tactile processing.
        config (dict): A configuration dictionary.
    """
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    logging.info(f"Using device: {device}")

    # --- Initialize Kinematics Solver ---
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
    
        kinematics_config = config.get("kinematics", {})
        solver = KinematicsSolver(
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

    logging.info("Kinematics solver initialized.")

    # --- Initialize Vision Processor ---
    vision_processor = VisionProcessor(config, device, data_dirs)
    logging.info("Vision processor initialized.")

    all_trajectories = []
    for data_dir in data_dirs:
        logging.info(f"📂 Processing dataset: {data_dir}")
        all_frame_dirs = load_frame_paths(data_dir)
        frame_names = [os.path.basename(f) for f in all_frame_dirs]
        
        clip_marks_path = Path(data_dir) / "clip_marks.json"
        if clip_marks_path.exists():
            with open(clip_marks_path, "r") as f:
                clip_marks = json.load(f)
            logging.info("Found clip_marks.json, processing clips as separate trajectories.")
            
            clips = clip_marks if isinstance(clip_marks, list) else clip_marks.values()
            for clip in clips:
                start_frame, end_frame = clip["start"], clip["end"]
                try:
                    i_start = frame_names.index(start_frame)
                    i_end = frame_names.index(end_frame)
                    
                    clip_frame_dirs = all_frame_dirs[i_start : i_end + 1]
                    ref_frame_dir = all_frame_dirs[i_start] # Use first frame of clip as reference
                    
                    trajectory_data = process_single_trajectory(
                        clip_frame_dirs, ref_frame_dir, vision_processor, solver, config
                    )
                    if trajectory_data is not None and isinstance(trajectory_data, dict):
                        all_trajectories.append(trajectory_data)

                except KeyboardInterrupt:
                    logging.info("KeyboardInterrupt detected. Finalizing dataset...")
                    break

                # except ValueError as e:
                #     logging.warning(f"Clip range ({start_frame} -> {end_frame}) not found in {data_dir}, skipping. Error: {e}")
        else:
            logging.info("No clip_marks.json found, processing entire directory as one trajectory.")
            ref_frame_dir = all_frame_dirs[0]
            trajectory_data = process_single_trajectory(all_frame_dirs, ref_frame_dir, vision_processor, solver, config)
            if trajectory_data is not None and isinstance(trajectory_data, dict):
                all_trajectories.append(trajectory_data)

    # --- Generate and print dataset summary ---
    generate_dataset_summary(all_trajectories, config, vision_processor)
    # log_detailed_tactile_analysis(all_trajectories)

    # --- Save the dataset as a list of trajectories ---
    # This format is crucial for correct splitting later on.
    out_path = Path(out_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "wb") as f:
        pickle.dump(all_trajectories, f)

    logging.info(f"✅ Successfully built dataset with {len(all_trajectories)} trajectories.")
    logging.info(f"💾 Saved dataset -> {out_path}")

def main():
    parser = argparse.ArgumentParser(description="Interactively build a robot learning dataset from raw demonstrations.")
    # The script is now primarily interactive, but we can keep args for advanced use or automation.
    parser.add_argument("--config", type=str, help="Optional: Directly provide a path to a config file to skip interactive selection.")
    args = parser.parse_args()

    try:
        # --- 1. Interactively Select Demonstration Datasets ---
        demo_choices = find_demo_dirs(paths.RAW_DATA_DIR)
        if not demo_choices:
            logging.error(f"No demonstration directories found in {paths.RAW_DATA_DIR}. Exiting."); return

        questions = [
            inquirer.Checkbox('selected_demos',
                              message="Select the demonstration datasets to build from (SPACE to select, ENTER to confirm)",
                              choices=demo_choices),
        ]
        answers = inquirer.prompt(questions)
        if not answers or not answers['selected_demos']:
            logging.info("No datasets selected. Exiting."); return
        selected_relative_paths = answers['selected_demos']
        data_dirs = [str(paths.WORKSPACE_ROOT / path_str) for path_str in selected_relative_paths]

        # --- 2. Interactively Select Configuration File ---
        if args.config:
            config_path_rel = args.config
            logging.info(f"Using provided config file: {config_path_rel}")
        else:
            config_choices = find_config_files(paths.CONFIG_DIR)
            if not config_choices:
                logging.error(f"No .yaml config files found in {paths.CONFIG_DIR}. Exiting."); return
            
            config_question = [
                inquirer.List('config_file',
                              message="Select the configuration file to use",
                              choices=config_choices),
            ]
            config_answer = inquirer.prompt(config_question)
            if not config_answer: logging.info("No config selected. Exiting."); return
            config_path_rel = config_answer['config_file']

        config_path_abs = paths.WORKSPACE_ROOT / config_path_rel
        with open(config_path_abs, 'r') as f:
            config = yaml.safe_load(f)
        logging.info(f"Loaded configuration from {config_path_abs}")
        
        # --- 3. Ask for Other Parameters ---
        other_questions = [
            inquirer.Text('out_file',
                          message="Enter the name for the output dataset file",
                          default="processed_dataset.pkl"),
            # inquirer.Confirm('height_map',
            #                  message="Use height map for tactile processing?",
            #                  default=True),
        ]
        other_answers = inquirer.prompt(other_questions)
        if not other_answers: logging.info("Cancelled. Exiting."); return
        
        # Construct the full output path
        out_file_path = paths.PROCESSED_DATA_DIR / other_answers['out_file']

    except (KeyboardInterrupt, TypeError):
        logging.info("\nDataset building cancelled by user.")
        return

    # Resolve paths within the config to be absolute
    if config.get("vision", {}).get("use_keypoint_extractor", False):
        yolo_path_str = config["vision"]["yolo_model_path"]
        abs_yolo_path = paths.WORKSPACE_ROOT / yolo_path_str
        if not abs_yolo_path.exists():
            logging.error(f"YOLO model not found at resolved path: {abs_yolo_path}"); return
        config["vision"]["yolo_model_path"] = str(abs_yolo_path)

    build_dataset(data_dirs, str(out_file_path), config)

if __name__ == "__main__":
    main()
