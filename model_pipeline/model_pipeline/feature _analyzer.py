import numpy as np
import os
import torch
import logging
import sys

# Set up logging for clearer output
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

# --- CONFIGURATION (!!! ADJUST THESE PATHS AND THRESHOLDS !!!) ---
ROLLOUT_DATA_PATH = "/home/user/franka_ros2_ws/models/rollout/rollout_mlp_joint_space.npz" # Update this filename
CHECKPOINT_PATH = "/home/user/franka_ros2_ws/models/policy_models/policy_mlp_joint_space_arm23.pt" # Update this path
Z_SCORE_THRESHOLD = 3.0
# Set to None to analyze the full rollout
ANALYZE_UP_TO_TIMESTEP = None 

# Define dimensions based on the provided feature construction:
DIM_TACTILE = 24    # 3 sensors * 8 features/sensor
DIM_ARM_PROP = 7
DIM_HAND_PROP = 16  # <-- Confirmed: 16D Hand Proprio
DIM_KP = 10         # 2 objects * 5 features/object
DIM_RGB_EMBED = 128
DIM_DEPTH_EMBED = 32
DIM_EMBED = DIM_RGB_EMBED + DIM_DEPTH_EMBED # 160

# --- DYNAMIC START INDICES ---
TACTILE_START = 0
ARM_PROP_START = TACTILE_START + DIM_TACTILE                 # 24
HAND_PROP_START = ARM_PROP_START + DIM_ARM_PROP              # 31
CAM1_KP_START = HAND_PROP_START + DIM_HAND_PROP              # 47
CAM1_EMBED_START = CAM1_KP_START + DIM_KP                    # 57
CAM2_KP_START = CAM1_EMBED_START + DIM_EMBED                 # 217
CAM2_EMBED_START = CAM2_KP_START + DIM_KP                    # 227

TOTAL_EXPECTED_DIM = CAM2_EMBED_START + DIM_EMBED # 387

# --- FEATURE MAPPING (Uses dynamic constants for self-tuning) ---
FEATURE_MAPPING = {
    # --- Tactile Features (24D: 3 Sensors * 8 Features/Sensor) ---
    # Order: [Centroid_x, Centroid_y, Major_x, Major_y, log(Eig_Major+1), log(Eig_Minor+1), Force_Proxy, Contact_Flag]
    "Tactile_Index_Tip_Centroid_X": (TACTILE_START + 0, TACTILE_START + 1),
    "Tactile_Index_Tip_Centroid_Y": (TACTILE_START + 1, TACTILE_START + 2),
    "Tactile_Index_Tip_Major_X": (TACTILE_START + 2, TACTILE_START + 3),
    "Tactile_Index_Tip_Major_Y": (TACTILE_START + 3, TACTILE_START + 4),
    "Tactile_Index_Tip_Major_Eig": (TACTILE_START + 4, TACTILE_START + 5),
    "Tactile_Index_Tip_Minor_Eig": (TACTILE_START + 5, TACTILE_START + 6),
    "Tactile_Index_Tip_Force_Proxy": (TACTILE_START + 6, TACTILE_START + 7),
    "Tactile_Index_Tip_Contact_Flag": (TACTILE_START + 7, TACTILE_START + 8),

    "Tactile_Middle_Tip_Centroid_X": (TACTILE_START + 8, TACTILE_START + 9),
    "Tactile_Middle_Tip_Centroid_Y": (TACTILE_START + 9, TACTILE_START + 10),
    "Tactile_Middle_Tip_Major_X": (TACTILE_START + 10, TACTILE_START + 11),
    "Tactile_Middle_Tip_Major_Y": (TACTILE_START + 11, TACTILE_START + 12),
    "Tactile_Middle_Tip_Major_Eig": (TACTILE_START + 12, TACTILE_START + 13),
    "Tactile_Middle_Tip_Minor_Eig": (TACTILE_START + 13, TACTILE_START + 14),
    "Tactile_Middle_Tip_Force_Proxy": (TACTILE_START + 14, TACTILE_START + 15),
    "Tactile_Middle_Tip_Contact_Flag": (TACTILE_START + 15, TACTILE_START + 16),
    
    "Tactile_Thumb_Tip_Centroid_X": (TACTILE_START + 16, TACTILE_START + 17),
    "Tactile_Thumb_Tip_Centroid_Y": (TACTILE_START + 17, TACTILE_START + 18),
    "Tactile_Thumb_Tip_Major_X": (TACTILE_START + 18, TACTILE_START + 19),
    "Tactile_Thumb_Tip_Major_Y": (TACTILE_START + 19, TACTILE_START + 20),
    "Tactile_Thumb_Tip_Major_Eig": (TACTILE_START + 20, TACTILE_START + 21),
    "Tactile_Thumb_Tip_Minor_Eig": (TACTILE_START + 21, TACTILE_START + 22),
    "Tactile_Thumb_Tip_Force_Proxy": (TACTILE_START + 22, TACTILE_START + 23),
    "Tactile_Thumb_Tip_Contact_Flag": (TACTILE_START + 23, TACTILE_START + 24),


    # --- Proprioception Features (23D) ---
    "Arm_Proprio_Joints_or_Pose": (ARM_PROP_START, ARM_PROP_START + DIM_ARM_PROP),  # Indices 24-30 (7D)
    "Hand_Proprio_Leap": (HAND_PROP_START, HAND_PROP_START + DIM_HAND_PROP),          # Indices 31-46 (16D)


    # --- Visual Features (340D) ---
    # Cam 1 (Indices 47 to 216)
    "Visual_Cam1_KP_Tube_Pos": (CAM1_KP_START + 0, CAM1_KP_START + 2),     # 2D Position (u, v)
    "Visual_Cam1_KP_Tube_Conf": (CAM1_KP_START + 2, CAM1_KP_START + 3),    # Confidence
    "Visual_Cam1_KP_Tube_Flag": (CAM1_KP_START + 3, CAM1_KP_START + 4),    # Detection Flag (Index 50)
    "Visual_Cam1_KP_Peg_Pos": (CAM1_KP_START + 4, CAM1_KP_START + 6),      # 2D Position (u, v)
    "Visual_Cam1_KP_Peg_Conf": (CAM1_KP_START + 6, CAM1_KP_START + 7),     # Confidence
    "Visual_Cam1_KP_Peg_Flag": (CAM1_KP_START + 7, CAM1_KP_START + 8),     # Detection Flag
    "Visual_Cam1_KP_Relative_Vec": (CAM1_KP_START + 8, CAM1_KP_START + 10), # 2D Relative Vector (tube - peg)
    
    "Visual_Cam1_RGB_Embed": (CAM1_EMBED_START, CAM1_EMBED_START + DIM_RGB_EMBED),
    "Visual_Cam1_Depth_Embed": (CAM1_EMBED_START + DIM_RGB_EMBED, CAM1_EMBED_START + DIM_EMBED),

    # Cam 2 (Indices 217 to 386)
    "Visual_Cam2_KP_Tube_Pos": (CAM2_KP_START + 0, CAM2_KP_START + 2),
    "Visual_Cam2_KP_Tube_Conf": (CAM2_KP_START + 2, CAM2_KP_START + 3),
    "Visual_Cam2_KP_Tube_Flag": (CAM2_KP_START + 3, CAM2_KP_START + 4),    # Detection Flag (Index 220)
    "Visual_Cam2_KP_Peg_Pos": (CAM2_KP_START + 4, CAM2_KP_START + 6),
    "Visual_Cam2_KP_Peg_Conf": (CAM2_KP_START + 6, CAM2_KP_START + 7),
    "Visual_Cam2_KP_Peg_Flag": (CAM2_KP_START + 7, CAM2_KP_START + 8),
    "Visual_Cam2_KP_Relative_Vec": (CAM2_KP_START + 8, CAM2_KP_START + 10),
    
    "Visual_Cam2_RGB_Embed": (CAM2_EMBED_START, CAM2_EMBED_START + DIM_RGB_EMBED),
    "Visual_Cam2_Depth_Embed": (CAM2_EMBED_START + DIM_RGB_EMBED, CAM2_EMBED_START + DIM_EMBED),
}


def load_normalization_data(checkpoint_path):
    """Loads mean and std vectors from the training checkpoint."""
    try:
        logging.info(f"Loading checkpoint from: {checkpoint_path}")
        # Load the checkpoint without expecting a model structure
        checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=False) 
        # --- FIX: Convert tensors to numpy arrays ---
        X_mean = checkpoint["X_mean"]
        X_std = checkpoint["X_std"]
        
        # Ensure they are numpy arrays if loaded as tensors
        if isinstance(X_mean, torch.Tensor):
            X_mean = X_mean.numpy()
        if isinstance(X_std, torch.Tensor):
            X_std = X_std.numpy()
            
        logging.info(f"Loaded normalization data for {len(X_mean)} dimensions.")
        return X_mean, X_std
    except Exception as e:
        logging.error(f"Error loading checkpoint or normalization constants from {checkpoint_path}: {e}")
        return None, None

def get_group_stats(inputs, inputs_normalized, X_mean, X_std):
    """Calculates descriptive statistics and deviation counts for each feature group."""
    stats = []
    
    for group_name, (start, end) in FEATURE_MAPPING.items():
        if start >= inputs.shape[1] or end > inputs.shape[1]:
            continue

        # Slice data for the current group
        group_data = inputs[:, start:end]
        if group_data.size == 0:
            continue
        group_norm_data = inputs_normalized[:, start:end]
        
        # 1. Descriptive Stats (on UN-NORMALIZED data)
        mean_val = np.mean(group_data)
        std_val = np.std(group_data)
        min_val = np.min(group_data)
        max_val = np.max(group_data)

        # 2. Deviation Count (on NORMALIZED data)
        deviation_count = np.sum(np.abs(group_norm_data) > Z_SCORE_THRESHOLD)
        group_dim = end - start
        
        stats.append({
            'name': group_name,
            'dim': group_dim,
            'mean': mean_val,
            'std': std_val,
            'min': min_val,
            'max': max_val,
            'dev_count': deviation_count
        })
        
    return stats

def print_detailed_analysis(inputs, inputs_normalized, X_mean, X_std, group_name, start_index, end_index, rollout_timesteps):
    """Prints a per-dimension analysis for a specific feature group."""
    
    actual_end = min(end_index, inputs.shape[1])
    if start_index >= actual_end:
        return

    group_norm_data = inputs_normalized[:, start_index:actual_end]
    group_data = inputs[:, start_index:actual_end]
    group_mean = X_mean[start_index:actual_end]
    group_std = X_std[start_index:actual_end]

    # Calculate deviations where normalized value exceeds the Z-score threshold
    deviation_counts_per_dim = np.sum(np.abs(group_norm_data) > Z_SCORE_THRESHOLD, axis=0)
    
    print(f"\n--- {group_name} (Indices {start_index} to {actual_end-1}, Dim: {actual_end - start_index}D) ---")
    print(f"{'Abs Index':<10}{'Dev Count':<12}{'Train Mean':<15}{'Train STD':<15}{'Rollout Min/Max':<25}")
    print("-" * 100)
    
    for i, count in enumerate(deviation_counts_per_dim):
        abs_index = start_index + i
        
        rollout_min_val = np.min(group_data[:, i])
        rollout_max_val = np.max(group_data[:, i])
        
        min_max_str = f"{rollout_min_val:.4f}/{rollout_max_val:.4f}"

        highlight = ""
        # The key logic for finding the constant bias
        if count == rollout_timesteps:
            highlight = " <--- **CONSTANT BIAS** (High Priority, likely bad feature)"
        elif count > rollout_timesteps * 0.5:
             highlight = " <--- High Deviation (> 50% of steps)"
        
        print(
            f"{abs_index:<10}"
            f"{count:<12}"
            f"{group_mean[i]:<15.4f}"
            f"{group_std[i]:<15.4f}"
            f"{min_max_str:<25}" 
            f"{highlight}"
        )

def report_visual_embedding_stats(inputs_normalized, cam_name, rgb_start, rgb_end):
    """
    Calculates the distribution statistics for the 128D visual embedding features.
    
    The L2 Norm (RMS) of the normalized vector should be close to sqrt(128) = 11.31 
    if the rollout data perfectly matches the training distribution.
    """
    actual_end = min(rgb_end, inputs_normalized.shape[1])
    if rgb_start >= actual_end:
        return

    group_norm_data = inputs_normalized[:, rgb_start:actual_end]
    rollout_timesteps = group_norm_data.shape[0]
    dim = actual_end - rgb_start # Should be 128
    
    # 1. Calculate the magnitude (L2 Norm) of the normalized vector for each timestep
    # L2 Norm = sqrt(sum(Z^2))
    normalized_l2_norms = np.sqrt(np.sum(group_norm_data**2, axis=1))
    
    # 2. Calculate the Expected L2 Norm (RMS)
    # The expected RMS magnitude for a 128D vector where each dimension is N(0, 1) is sqrt(dim)
    expected_rms_magnitude = np.sqrt(dim)
    
    # 3. Calculate the actual statistics of the L2 norms
    mean_l2 = np.mean(normalized_l2_norms)
    std_l2 = np.std(normalized_l2_norms)
    max_l2 = np.max(normalized_l2_norms)
    
    # 4. Calculate Max Z-Score across all dimensions in this group
    max_z_score = np.max(np.abs(group_norm_data))
    
    print(f"\n--- VISUAL RGB EMBEDDING DISTRIBUTION: {cam_name} (Indices {rgb_start} to {actual_end-1}) ---")
    print(f"Dimension: {dim}D, Expected Normalized RMS Magnitude (sqrt({dim})): {expected_rms_magnitude:.3f}")
    print("-" * 80)
    print(f"{'Metric':<25}{'Value':<15}{'Deviation Status'}")
    print("-" * 80)
    print(f"{'Mean Normalized L2 Norm':<25}{mean_l2:<15.3f}{' (Should be close to Expected RMS)'}")
    print(f"{'Max Normalized L2 Norm':<25}{max_l2:<15.3f}{f' (Max distance from origin in {dim}D space)'}")
    print(f"{'Max Z-Score (Single Dim)':<25}{max_z_score:<15.3f}{f' (Max outlier in a single dimension)'}")
    print("-" * 80)
    
    # Compare the mean L2 norm to the expected RMS
    l2_ratio = mean_l2 / expected_rms_magnitude
    if l2_ratio > 1.2 or l2_ratio < 0.8: # Check if it deviates > 20%
        logging.warning(f"⚠️ Mean L2 Norm is {l2_ratio:.2f}x the expected value. The live visual distribution is significantly shifted!")
    elif l2_ratio > 1.1 or l2_ratio < 0.9:
        logging.warning(f"⚠️ Mean L2 Norm is {l2_ratio:.2f}x the expected value. Minor shift detected.")
    else:
        logging.info(f"✅ Mean L2 Norm is {l2_ratio:.2f}x the expected value. Visual embedding distribution is centered reasonably well.")
        
    print("-" * 80)


def analyze_input_deviations():

    if not os.path.exists(ROLLOUT_DATA_PATH):
        logging.error(f"Rollout data not found at: {ROLLOUT_DATA_PATH}")
        return

    try:
        data = np.load(ROLLOUT_DATA_PATH)
        inputs = data['inputs']
    except Exception as e:
        logging.error(f"Error loading or reading inputs from NPZ: {e}")
        return

    X_mean, X_std = load_normalization_data(CHECKPOINT_PATH)
    if X_mean is None or X_std is None:
        return

    rollout_timesteps = inputs.shape[0]
    logging.info(f"Loaded {rollout_timesteps} input vectors, each with dimension {inputs.shape[1]}.")
    
    if ANALYZE_UP_TO_TIMESTEP is not None:
        if ANALYZE_UP_TO_TIMESTEP > rollout_timesteps:
            logging.warning(f"ANALYZE_UP_TO_TIMESTEP ({ANALYZE_UP_TO_TIMESTEP}) is > total timesteps ({rollout_timesteps}). Using all timesteps.")
            rollout_timesteps = rollout_timesteps
        else:
            logging.info(f"--- TRUNCATING ANALYSIS: Analyzing first {ANALYZE_UP_TO_TIMESTEP} of {rollout_timesteps} timesteps. ---")
            inputs = inputs[:ANALYZE_UP_TO_TIMESTEP]
            rollout_timesteps = inputs.shape[0] # This is now the new, smaller number
    else:
        rollout_timesteps = rollout_timesteps # Use all timesteps


    if inputs.shape[1] != TOTAL_EXPECTED_DIM:
        logging.warning(f"DIMENSION MISMATCH: Input vectors are {inputs.shape[1]}D, but EXPECTED_DIM is {TOTAL_EXPECTED_DIM}D. Check the map constants.")
    else:
        logging.info(f"Dimension check passed: Input is {inputs.shape[1]}D (Matches {TOTAL_EXPECTED_DIM}D expected).")
        
    # Prepare normalized inputs, handling division by zero for STD
    X_std_safe = np.where(X_std == 0, 1e-8, X_std) 
    
    # Check if mean/std match input shape before normalization
    if X_mean.shape[0] < inputs.shape[1] or X_std_safe.shape[0] < inputs.shape[1]:
        logging.error(f"Normalization vectors (len={X_mean.shape[0]}) are shorter than input vectors (len={inputs.shape[1]}). Cannot perform analysis.")
        return
        
    inputs_normalized = (inputs - X_mean[:inputs.shape[1]]) / X_std_safe[:inputs.shape[1]]
      
    print("\n" + "=" * 100)
    print(f"--- COMPLETE FEATURE DEVIATION ANALYSIS ---")
    print(f"Total Timesteps: {rollout_timesteps}, Z-Score Threshold: Z > {Z_SCORE_THRESHOLD:.1f}")
    print(f"Total Expected Dimensions: {TOTAL_EXPECTED_DIM}")
    print("=" * 100)
    
    # Cam 1 RGB Embed Inspection
    cam1_rgb_start, _ = FEATURE_MAPPING["Visual_Cam1_RGB_Embed"]
    inspect_raw_visual_data(inputs, X_mean, X_std_safe, "CAM 1 RGB EMBEDDING", cam1_rgb_start)

    # Cam 2 RGB Embed Inspection
    cam2_rgb_start, _ = FEATURE_MAPPING["Visual_Cam2_RGB_Embed"]
    inspect_raw_visual_data(inputs, X_mean, X_std_safe, "CAM 2 RGB EMBEDDING", cam2_rgb_start)

    # -------------------------------------------------------------------
    # REPORT 1: VISUAL EMBEDDING DISTRIBUTION REPORT (New!)
    # -------------------------------------------------------------------
    
    print("\n" + "#" * 30 + " VISUAL EMBEDDING DISTRIBUTION REPORT " + "#" * 30)
    
    # Cam 1 RGB Embed
    cam1_rgb_start, cam1_rgb_end = FEATURE_MAPPING["Visual_Cam1_RGB_Embed"]
    report_visual_embedding_stats(inputs_normalized, "CAM 1 RGB EMBEDDING", cam1_rgb_start, cam1_rgb_end)

    # Cam 2 RGB Embed
    cam2_rgb_start, cam2_rgb_end = FEATURE_MAPPING["Visual_Cam2_RGB_Embed"]
    report_visual_embedding_stats(inputs_normalized, "CAM 2 RGB EMBEDDING", cam2_rgb_start, cam2_rgb_end)

    # -------------------------------------------------------------------
    # REPORT 2: AGGREGATE GROUP STATISTICS (High-Level Summary)
    # -------------------------------------------------------------------
    
    print("\n" + "#" * 30 + " AGGREGATE GROUP STATISTICS " + "#" * 30)
    print("Provides a high-level overview of each feature group's behavior during rollout.")

    group_stats = get_group_stats(inputs, inputs_normalized, X_mean, X_std)
    
    print("\n" + "=" * 85)
    print("FEATURE GROUP STATISTICS & DEVIATION COUNT (Un-normalized values, Z-Score threshold = 3.0)")
    print("=" * 85)
    
    header = f"{'Feature Group':<35}{'Dim':<5}{'Mean':<10}{'Std Dev':<10}{'Min':<10}{'Max':<10}{'Deviations (>3.0)':<18}"
    print(header)
    print("-" * 85)
    
    for stat in group_stats:
        print(f"{stat['name']:<35}{stat['dim']:<5}{stat['mean']:<10.3f}{stat['std']:<10.3f}{stat['min']:<10.3f}{stat['max']:<10.3f}{stat['dev_count']:<18}")
    
    print("-" * 85)

    # -------------------------------------------------------------------
    # REPORT 3: TOP MAX DEVIATION REPORT (Worst Offenders)
    # -------------------------------------------------------------------

    print("\n" + "#" * 30 + " TOP MAX DEVIATION REPORT " + "#" * 30)
    print("Shows the single worst Z-Score that occurred in each group, sorted by severity.")

    max_abs_z_scores = np.max(np.abs(inputs_normalized), axis=0)
    all_deviations = []

    for group_name, (start, end) in FEATURE_MAPPING.items():
        if start >= inputs.shape[1] or end > inputs.shape[1]:
            continue 
            
        group_max_z = max_abs_z_scores[start:end]
        
        if group_max_z.size == 0:
             continue
             
        max_idx_relative = np.argmax(group_max_z)
        max_z_score = group_max_z[max_idx_relative]
        
        if max_z_score > Z_SCORE_THRESHOLD:
            max_idx_absolute = start + max_idx_relative
            deviating_timestep = np.argmax(np.abs(inputs_normalized[:, max_idx_absolute]))
            original_value = inputs[deviating_timestep, max_idx_absolute]
            
            all_deviations.append({
                'name': group_name,
                'index': max_idx_absolute,
                'z_score': max_z_score,
                'timestep': deviating_timestep,
                'value': original_value
            })

    if not all_deviations:
        print(f"✅ No feature group exceeded the Max Z-score threshold of {Z_SCORE_THRESHOLD:.1f} during this rollout.")
    else:
        all_deviations.sort(key=lambda x: x['z_score'], reverse=True)
        
        print(f"{'Z-SCORE (Max)':<15}{'Index (Abs)':<15}{'Timestep':<10}{'Value':<10}{'Feature Group'}")
        print("-" * 75)
        for dev in all_deviations:
            print(f"{dev['z_score']:<15.2f}{dev['index']:<15}{dev['timestep']:<10}{dev['value']:<10.3f}{dev['name']}")

    # -------------------------------------------------------------------
    # REPORT 4: DETAILED PER-DIMENSION ANALYSIS (The Deep Dive)
    # -------------------------------------------------------------------
    
    print("\n" + "#" * 30 + " DETAILED PER-DIMENSION ANALYSIS " + "#" * 30)
    print("Provides a line-by-line breakdown for low-dimensional features.")
    
    low_dim_groups = [
        "Arm_Proprio_Joints_or_Pose", "Hand_Proprio_Leap",
        "Tactile_Index_Tip_Centroid_X", "Tactile_Index_Tip_Centroid_Y", "Tactile_Index_Tip_Major_X", "Tactile_Index_Tip_Major_Y", "Tactile_Index_Tip_Major_Eig", "Tactile_Index_Tip_Minor_Eig", "Tactile_Index_Tip_Force_Proxy", "Tactile_Index_Tip_Contact_Flag",
        "Tactile_Middle_Tip_Centroid_X", "Tactile_Middle_Tip_Centroid_Y", "Tactile_Middle_Tip_Major_X", "Tactile_Middle_Tip_Major_Y", "Tactile_Middle_Tip_Major_Eig", "Tactile_Middle_Tip_Minor_Eig", "Tactile_Middle_Tip_Force_Proxy", "Tactile_Middle_Tip_Contact_Flag",
        "Tactile_Thumb_Tip_Centroid_X", "Tactile_Thumb_Tip_Centroid_Y", "Tactile_Thumb_Tip_Major_X", "Tactile_Thumb_Tip_Major_Y", "Tactile_Thumb_Tip_Major_Eig", "Tactile_Thumb_Tip_Minor_Eig", "Tactile_Thumb_Tip_Force_Proxy", "Tactile_Thumb_Tip_Contact_Flag",
        "Visual_Cam1_KP_Tube_Pos", "Visual_Cam1_KP_Tube_Conf", "Visual_Cam1_KP_Tube_Flag", "Visual_Cam1_KP_Peg_Pos", "Visual_Cam1_KP_Peg_Conf", "Visual_Cam1_KP_Peg_Flag", "Visual_Cam1_KP_Relative_Vec",
        "Visual_Cam2_KP_Tube_Pos", "Visual_Cam2_KP_Tube_Conf", "Visual_Cam2_KP_Tube_Flag", "Visual_Cam2_KP_Peg_Pos", "Visual_Cam2_KP_Peg_Conf", "Visual_Cam2_KP_Peg_Flag", "Visual_Cam2_KP_Relative_Vec"
    ]

    for group_name in low_dim_groups:
        start, end = FEATURE_MAPPING[group_name]
        print_detailed_analysis(inputs, inputs_normalized, X_mean, X_std_safe, 
                                group_name, start, end, rollout_timesteps)

    print("\n" + "=" * 100)
    print("--- Analysis Complete ---")

def inspect_raw_visual_data(inputs, X_mean, X_std, cam_name, rgb_start):
    """Prints raw feature values and constants for the first 5 dimensions of an embedding."""
    
    dim_count = 5 # Just inspect the first 5 dimensions
    actual_end = min(rgb_start + dim_count, inputs.shape[1])
    
    if rgb_start >= actual_end:
        return
        
    print(f"\n--- RAW DATA INSPECTION: {cam_name} (Indices {rgb_start} to {actual_end-1}) ---")
    
    # Slice the data
    group_data = inputs[:2, rgb_start:actual_end] # First 2 timesteps
    group_mean = X_mean[rgb_start:actual_end]
    group_std = X_std[rgb_start:actual_end]

    print(f"{'Index':<8}{'Train Mean':<12}{'Train STD':<12}{'Input T=0':<12}{'Input T=1':<12}")
    print("-" * 55)
    
    for i in range(group_data.shape[1]):
        abs_index = rgb_start + i
        mean_val = group_mean[i]
        std_val = group_std[i]
        t0_val = group_data[0, i]
        t1_val = group_data[1, i]

        # Calculate Z-score for T=0
        if std_val > 1e-8:
            z_score_t0 = (t0_val - mean_val) / std_val
        else:
            z_score_t0 = 0.0

        highlight = ""
        if abs(z_score_t0) > Z_SCORE_THRESHOLD:
            highlight = f" (Z={z_score_t0:.1f}!)"

        print(
            f"{abs_index:<8}"
            f"{mean_val:<12.4f}"
            f"{std_val:<12.4f}"
            f"{t0_val:<12.4f}"
            f"{t1_val:<12.4f}"
            f"{highlight}"
        )

if __name__ == '__main__':
    analyze_input_deviations()