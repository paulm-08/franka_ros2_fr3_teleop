"""
dataset_builder.py (Revised)

- Processes raw multimodal data (tactile, visual, proprio) from demonstrations.
- Uses the revised tactile_features script to get rich 8D features.
- Correctly handles multiple clips and demonstrations as separate trajectories.
- Saves the final dataset as a list of trajectories in a .pkl file to preserve
  boundaries, which is critical for correct data splitting.
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

from model_pipeline.visual_embedder import VisualEmbedder
from model_pipeline.keypoint_extractor import KeypointExtractor
# Import the revised feature extractor and its dimension constant
from model_pipeline.tactile_features import process_tactile_image, TACTILE_FEATURE_DIM
from model_pipeline.utils import load_frame_paths, load_actions, get_cfg_path, init_sensor

# It's good practice to define sensor names for consistent ordering
SENSOR_ORDER = ["rindex", "rmiddle", "rthumb"]

# --- Logger Setup ---
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()]
)

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

def process_single_trajectory(frame_dirs, ref_frame_dir, use_height_map, embedder):
    """
    Processes a single continuous trajectory (a clip or a full demonstration).

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
    
    # --- Per-frame feature extraction ---
    tactile_list, visual_list = [], []
    for frame_dir in frame_dirs:
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
        
        # Generate embeddings for each modality
        rgb_emb1 = embedder.embed_rgb(color1)
        depth_emb1 = embedder.embed_depth(depth1)
        rgb_emb2 = embedder.embed_rgb(color2)
        depth_emb2 = embedder.embed_depth(depth2)

        # Create a combined feature vector for each camera
        # --- REFINED LOGIC ---
        # This robustly handles all cases: ResNet, Keypoints, or a mix
        all_camera_features = []

        # Process Camera 1
        cam1_feats = []
        if rgb_emb1 is not None: cam1_feats.append(rgb_emb1)
        if depth_emb1 is not None: cam1_feats.append(depth_emb1)
        if cam1_feats: all_camera_features.append(np.concatenate(cam1_feats))

        # Process Camera 2
        cam2_feats = []
        if rgb_emb2 is not None: cam2_feats.append(rgb_emb2)
        if depth_emb2 is not None: cam2_feats.append(depth_emb2)
        if cam2_feats: all_camera_features.append(np.concatenate(cam2_feats))

        # Average the feature vectors from all available and valid cameras
        if not all_camera_features:
            # Calculate total expected dimension if no cameras provide features
            total_dim = sum(dim for dim in embedder.out_dim.values() if dim > 0)
            visual_vec = np.zeros(total_dim, dtype=np.float32)
        else:
            visual_vec = np.mean(all_camera_features, axis=0).astype(np.float32)

        visual_list.append(visual_vec)

    # --- Convert lists to numpy arrays ---
    tactile_feats = np.array(tactile_list, dtype=np.float32)
    visual_feats = np.stack(visual_list, axis=0)

    # --- Create State-Action pairs ---
    # State at time t (s_t)
    joints_t = actions[:-1]
    tactile_t = tactile_feats[:-1]
    visual_t = visual_feats[:-1]

    # Action at time t (a_t)
    delta_q = actions[1:] - actions[:-1]

    # After concatenating tactile, visual, and joint features
    expected_dim = (len(SENSOR_ORDER) * TACTILE_FEATURE_DIM) + visual_vec.shape[0] + joints_t.shape[1]
    assert tactile_t.shape[1] + visual_t.shape[1] + joints_t.shape[1] == expected_dim, "State vector dimension mismatch!"

    return {
        "tactile_t": tactile_t,
        "visual_t": visual_t,
        "joints_t": joints_t,
        "delta_q": delta_q,
    }

def generate_dataset_summary(all_trajectories):
    """
    Calculates and prints a statistical summary of the entire dataset.
    """
    if not all_trajectories:
        logging.warning("Cannot generate summary for an empty dataset.")
        return

    logging.info("\n" + "="*60)
    logging.info("          🤖 DATASET STATISTICAL SUMMARY 🤖")
    logging.info("="*60)

    # --- Aggregate all data into single arrays for analysis ---
    all_tactile = np.concatenate([traj['tactile_t'] for traj in all_trajectories], axis=0)
    all_visual = np.concatenate([traj['visual_t'] for traj in all_trajectories], axis=0)
    all_joints = np.concatenate([traj['joints_t'] for traj in all_trajectories], axis=0)
    all_actions = np.concatenate([traj['delta_q'] for traj in all_trajectories], axis=0)
    traj_lengths = [len(traj['joints_t']) for traj in all_trajectories]
    
    total_samples = all_joints.shape[0]

    # --- Overall Summary ---
    logging.info(f"Number of trajectories: {len(all_trajectories)}")
    logging.info(f"Total number of samples (state-action pairs): {total_samples}")
    logging.info(f"Trajectory Lengths | Min: {min(traj_lengths)}, Max: {max(traj_lengths)}, Avg: {np.mean(traj_lengths):.1f}")

    # --- Per-Modality Summary ---
    def print_stats(name, data):
        logging.info(f"\n--- {name} (Shape: {data.shape}) ---")
        # Check for NaN or Inf values
        if np.isnan(data).any() or np.isinf(data).any():
            logging.error(f"  ❌ Found NaN or Inf values in {name} data!")
        else:
            logging.info(f"  ✅ No NaN or Inf values detected.")
        
        # Calculate and print stats
        min_vals = np.min(data, axis=0)
        max_vals = np.max(data, axis=0)
        mean_vals = np.mean(data, axis=0)
        std_vals = np.std(data, axis=0)
        
        logging.info(f"  Min value (overall): {np.min(min_vals):.4f}")
        logging.info(f"  Max value (overall): {np.max(max_vals):.4f}")
        logging.info(f"  Mean value (overall): {np.mean(mean_vals):.4f}")
        logging.info(f"  Std Dev (overall): {np.mean(std_vals):.4f}")

    print_stats("Tactile Features", all_tactile)
    print_stats("Visual Features", all_visual)
    print_stats("Joint States (Proprioception)", all_joints)
    print_stats("Joint Actions (delta_q)", all_actions)
    logging.info("="*60)

def build_dataset(data_dirs, out_file, use_height_map, config):
    """
    Builds a dataset from multiple raw data directories.

    Args:
        data_dirs (list): List of paths to the raw demonstration directories.
        out_file (str): Path to save the output .pkl file.
        use_height_map (bool): Whether to use the height map for tactile processing.
        config (dict): A configuration dictionary.
    """
    if config.get("global_depth_range"):
        logging.info(f"Using pre-computed depth range from config: {config['global_depth_range']}")
        global_depth_range = config["global_depth_range"]
    else:
        # Only compute if not already provided
        global_depth_range = compute_global_depth_range(data_dirs)
        
    device = "cuda" if torch.cuda.is_available() else "cpu"

    # --- Initialize the KeypointExtractor based on config ---
    use_extractor = config.get("vision", {}).get("use_keypoint_extractor", False)
    if use_extractor:
        logging.info("Using KeypointExtractor for visual features.")
        # This replaces the VisualEmbedder initialization
        use_3d_kps = config.get("vision", {}).get("use_3d", False)
        
        extractor = KeypointExtractor(
            model_path=config.get("vision", {}).get("yolo_model_path"),
            use_3d=use_3d_kps,
            # These will be ignored if use_3d is False, but are needed if True
            intrinsics_path=config.get("vision", {}).get("intrinsics_path"),
            extrinsics_path=config.get("vision", {}).get("extrinsics_path")
        )
        logging.info(f"Keypoint extractor initialized with output dimension={extractor.output_dim}")
        
        # Define a simple wrapper to match the embedder interface
        class SimpleEmbedder:
            def __init__(self, extractor):
                self.extractor = extractor
                self.out_dim = {'rgb': extractor.output_dim, 'depth': 0} # Depth not used here

            def embed_rgb(self, img):
                return self.extractor.extract_scene_features(img)

            def embed_depth(self, img):
                return None # Not used

        embedder = SimpleEmbedder(extractor)
    else:
        logging.info("Using VisualEmbedder for visual features.")
        # Define the output dimensions for RGB and Depth embeddings
        visual_dim = config.get("visual_dim", 256)
        depth_dim = config.get("depth_dim", 128)
        out_dim = {'rgb': visual_dim, 'depth': depth_dim}

        # The corrected call (just remove the out_dim line)
        embedder = VisualEmbedder(
            backbone=config.get("backbone", "resnet18"),
            device=device,
            pretrained=True,
            out_dim=out_dim,
            global_depth_range=global_depth_range
        )
        logging.info(f"Visual embedder initialized on {device} with out_dim={embedder.out_dim}")

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
                    
                    trajectory_data = process_single_trajectory(clip_frame_dirs, ref_frame_dir, use_height_map, embedder)
                    if trajectory_data:
                        all_trajectories.append(trajectory_data)

                except ValueError:
                    logging.warning(f"Clip range ({start_frame} -> {end_frame}) not found in {data_dir}, skipping.")
        else:
            logging.info("No clip_marks.json found, processing entire directory as one trajectory.")
            ref_frame_dir = all_frame_dirs[0]
            trajectory_data = process_single_trajectory(all_frame_dirs, ref_frame_dir, use_height_map, embedder)
            if trajectory_data:
                all_trajectories.append(trajectory_data)

    # --- Generate and print dataset summary ---
    generate_dataset_summary(all_trajectories)

    # --- Save the dataset as a list of trajectories ---
    # This format is crucial for correct splitting later on.
    out_path = Path(out_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "wb") as f:
        pickle.dump(all_trajectories, f)

    logging.info(f"✅ Successfully built dataset with {len(all_trajectories)} trajectories.")
    logging.info(f"💾 Saved dataset -> {out_path}")

def main():
    parser = argparse.ArgumentParser(description="Build a robot learning dataset from raw demonstrations.")
    # For simplicity, main arguments are here. For a real project, use a YAML config file.
    parser.add_argument("--data_dirs", nargs="+", required=True, help="List of dataset directories to process.")
    parser.add_argument("--out_file", type=str, required=True, help="Path to the output .pkl dataset file.")
    parser.add_argument("--config", type=str, default="config.yaml", help="Path to a YAML config file.")
    parser.add_argument("--height_map", action="store_true", help="Use 9DTact height map instead of raw image.")
    args = parser.parse_args()
    
    # In a real pipeline, you would load a config.yaml file here
    # For now, we'll use a default dictionary
    config = {
        "backbone": "resnet18",
        "visual_dim": 256,
        "global_depth_range": [154, 2826],
        # Add other params like frame stacking window K here
        # --- New settings for controlling vision module ---
        "vision": {
            "use_keypoint_extractor": True,  # Set to false to use ResNet18
            # --- KeypointExtractor settings (ignored if false) ---
            "yolo_model_path": "./runs/detect/yolov8_custom6/weights/best.pt",
            "use_3d": False,  # Set to true for 3D coordinates
            "intrinsics_path": "path/to/intrinsics_cam1.json",  # Needed for 3D
            "extrinsics_path": "path/to/T_base_cam1.npy"  # Needed for 3D
        }
    }

    data_dirs = [Path(d).expanduser().resolve() for d in args.data_dirs]
    for d in data_dirs:
        if not d.exists():
            logging.error(f"Data directory not found: {d}")
            return
            
    build_dataset(data_dirs, args.out_file, args.height_map, config)

if __name__ == "__main__":
    main()