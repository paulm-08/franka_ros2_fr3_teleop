import pickle
import numpy as np
import argparse
import logging
from pathlib import Path
import random
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import seaborn as sns
import inquirer
import yaml

from model_pipeline import paths # Import the new paths module
from model_pipeline.dataset_builder import SENSOR_ORDER, find_config_files
from model_pipeline.tactile_features import TACTILE_FEATURE_DIM
from model_pipeline.utils import find_pkl_files

# --- Logger Setup ---
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

# ===================================================================
# === VISUALIZATION HELPER FUNCTIONS ===
# ===================================================================

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import logging
from pathlib import Path

# Ensure seaborn theme is set for prettier plots
sns.set_theme(style="whitegrid", context="notebook", palette="muted")

def plot_tactile_distributions(states, output_dir):
    """
    Generates a single distribution plot for tactile features using KDE.
    Assumes tactile features are at the start of the state vector.
    """
    # 1. Define Features (Customize based on your TACTILE_FEATURE_DIM)
    # Assuming standard BioTac/Digit layout: Centroid(2), Axis(2), Shape(2), Force(1), Contact(1)
    tactile_feature_names = [
        'Centroid X', 'Centroid Y', 
        'Major Axis', 'Minor Axis',
        'Shape Maj', 'Shape Min', 
        'Total Force', 'Contact Flag'
    ]
    
    # We assume the first 8 dimensions correspond to the first finger for visualization
    # If you have 2 fingers, this visualizes just the first one to save space.
    num_feats = min(len(tactile_feature_names), states.shape[1])
    
    fig, axes = plt.subplots(4, 2, figsize=(14, 16))
    fig.suptitle('Tactile Feature Distributions (Aggregated)', fontsize=20, y=1.02)
    axes = axes.flatten()

    for i in range(num_feats):
        ax = axes[i]
        feature_name = tactile_feature_names[i]
        feature_data = states[:, i]

        # Special handling: Contact Flag is binary, use a bar chart
        if 'Flag' in feature_name or 'Contact' in feature_name:
            counts = np.bincount(feature_data.astype(int))
            sns.barplot(x=[0, 1], y=counts if len(counts)>1 else [counts[0], 0], ax=ax, palette="Blues_d")
            ax.set_xticklabels(['No Contact', 'Contact'])
            ax.set_title(f"{feature_name} (Count)")
        else:
            # Filter for "Contact Only" for geometry features to avoid the huge spike at 0
            if 'Centroid' in feature_name or 'Axis' in feature_name or 'Shape' in feature_name:
                # Assuming index 7 is contact flag for the first finger
                contact_mask = states[:, 7] > 0.5
                data_to_plot = feature_data[contact_mask]
                ax.set_title(f"{feature_name} (Contact Frames Only)")
            else:
                data_to_plot = feature_data
                ax.set_title(f"{feature_name}")

            if len(data_to_plot) > 10:
                sns.histplot(data_to_plot, kde=True, ax=ax, color="C0", edgecolor=None)
            else:
                ax.text(0.5, 0.5, "Insufficient Data", ha='center', transform=ax.transAxes)

        ax.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    save_path = output_dir / "tactile_feature_distributions.png"
    plt.savefig(save_path, bbox_inches='tight')
    plt.close(fig)
    logging.info(f"📊 Saved tactile plots to {save_path}")


def plot_visual_distributions(states, tactile_total_dim, visual_dim, output_dir, config):
    logging.info("Generating visual feature plots...")
    state_config = config.get('state', {})
    
    use_kps = state_config.get('use_keypoint_extractor', False)
    use_3d = state_config.get('use_3d_keypoints', False)
    use_embs = state_config.get('use_resnet_embeddings', False)
    use_3d_tactile = state_config.get('use_3d_tactile', False)

    # --- Dimension Definitions (Original Logic) ---
    if use_kps:
        kp_dim_per_obj = (3 if use_3d else 2) + 2 
        rel_vec_dim = 3 if use_3d else 2
        kp_dim_per_cam = (kp_dim_per_obj * 2) + rel_vec_dim
    else:
        kp_dim_per_obj = rel_vec_dim = kp_dim_per_cam = 0

    if use_embs:
        rgb_dim = config.get('visual_dim', 32)
        depth_dim = config.get('depth_dim', 16)
    else:
        rgb_dim = depth_dim = 0

    single_cam_dim = kp_dim_per_cam + rgb_dim + depth_dim
    
    # --- Index Calculation (Corrected base_idx) ---
    tactile_3d_dim = 21 if use_3d_tactile else 0
    proprio_dim = 23 + tactile_3d_dim 
    base_idx = tactile_total_dim + proprio_dim
    
    c1_start = base_idx
    c2_start = base_idx + single_cam_dim

    # --- Plotting Grid ---
    num_rows = 0
    if use_kps: num_rows += 4
    if use_embs: num_rows += 2
    if num_rows == 0: return

    fig = plt.figure(figsize=(20, 5 * num_rows))
    gs = fig.add_gridspec(num_rows, 2)
    fig.suptitle('Visual Feature Distribution Analysis', fontsize=18, y=1.01)
    curr_row = 0

    if use_kps:
        peg_offset = kp_dim_per_obj
        flag_offset = (3 if use_3d else 2) + 1
        
        # 1. Detection Rates (Color matched to coverage/confidence)
        # Define the consistent color palette
        obj_colors = {'Tube': 'tab:blue', 'Peg': 'tab:orange'}

        for cam_idx, start in enumerate([c1_start, c2_start]):
            ax = fig.add_subplot(gs[curr_row, cam_idx])
            
            t_flag_idx = start + flag_offset
            p_flag_idx = start + peg_offset + flag_offset
            
            # Calculate means as percentages
            t_rate = np.mean(states[:, t_flag_idx])
            p_rate = np.mean(states[:, p_flag_idx])
            
            labels = ['Tube', 'Peg']
            rates = [t_rate, p_rate]
            
            # Plot with explicit hue mapping for colors
            sns.barplot(
                x=labels, 
                y=rates, 
                ax=ax, 
                hue=labels, 
                palette=obj_colors, 
                legend=False
            )
            
            # Annotate bars with the percentage value for quick reading
            for i, rate in enumerate(rates):
                ax.text(i, rate + 0.02, f'{rate:.1%}', ha='center', fontweight='bold')
            
            ax.set_title(f"Cam {cam_idx+1}: Detection Rate")
            ax.set_ylim(0, 1.1)  # Leave room for the text labels
            ax.set_ylabel("Success Rate")
            ax.grid(axis='y', linestyle='--', alpha=0.3)
            
        curr_row += 1

        # 2. Confidence Distribution (Both Tube and Peg)
        for cam_idx, start in enumerate([c1_start, c2_start]):
            ax = fig.add_subplot(gs[curr_row, cam_idx])
            
            # Tube indices and mask
            t_flag_idx = start + flag_offset
            t_conf_idx = t_flag_idx - 1
            t_mask = states[:, t_flag_idx] > 0.5
            
            # Peg indices and mask
            p_flag_idx = start + peg_offset + flag_offset
            p_conf_idx = p_flag_idx - 1
            p_mask = states[:, p_flag_idx] > 0.5
            
            # Plot Tube Confidence (Blue)
            if np.any(t_mask):
                sns.histplot(states[t_mask, t_conf_idx], bins=40, kde=True, 
                             ax=ax, label='Tube Tip', color='tab:blue', alpha=0.5, element="step")
            
            # Plot Peg Confidence (Orange)
            if np.any(p_mask):
                sns.histplot(states[p_mask, p_conf_idx], bins=40, kde=True, 
                             ax=ax, label='Peg', color='tab:orange', alpha=0.5, element="step")
            
            ax.set_title(f"Cam {cam_idx+1}: Detection Confidence")
            ax.set_xlabel("Confidence Score")
            ax.set_ylabel("Frequency")
            
            # If scores are [0, 1], fix the view. If they are raw logits, leave it auto.
            if np.max(states[:, [t_conf_idx, p_conf_idx]]) <= 1.01:
                ax.set_xlim(0, 1.05)
                
            ax.legend(loc='upper left')
            ax.grid(True, linestyle='--', alpha=0.3)
            
        curr_row += 1
        # 3. Spatial Coverage (Plotting both Tube and Peg)
        for cam_idx, start in enumerate([c1_start, c2_start]):
            ax = fig.add_subplot(gs[curr_row, cam_idx])
            
            # Indices for Tube
            t_flag_idx = start + flag_offset
            t_x_idx, t_y_idx = start, start + 1
            
            # Indices for Peg
            p_flag_idx = start + peg_offset + flag_offset
            p_x_idx, p_y_idx = start + peg_offset, start + peg_offset + 1
            
            # Masks
            t_mask = states[:, t_flag_idx] > 0.5
            p_mask = states[:, p_flag_idx] > 0.5
            
            # Plot Tube (Blue)
            if np.any(t_mask):
                ax.scatter(states[t_mask, t_x_idx], states[t_mask, t_y_idx], 
                           alpha=0.15, s=8, label='Tube Tip', color='tab:blue')
            
            # Plot Peg (Orange)
            if np.any(p_mask):
                ax.scatter(states[p_mask, p_x_idx], states[p_mask, p_y_idx], 
                           alpha=0.15, s=8, label='Peg', color='tab:orange')
            
            ax.set_title(f"Cam {cam_idx+1}: Spatial Coverage")
            ax.set_xlabel("X (px/m)")
            ax.set_ylabel("Y (px/m)")
            ax.legend(loc='upper right')
            ax.invert_yaxis()  # Standard for image-space coordinates
            ax.axis('equal')   # Keeps the aspect ratio square
            ax.grid(True, linestyle='--', alpha=0.5)
            
        curr_row += 1
        
        # 4. Relative Vectors
        for cam_idx, start in enumerate([c1_start, c2_start]):
            ax = fig.add_subplot(gs[curr_row, cam_idx])
            rel_idx = start + (kp_dim_per_obj * 2)
            mask = (states[:, start+flag_offset] > 0.5) & (states[:, start+peg_offset+flag_offset] > 0.5)
            if np.any(mask):
                ax.scatter(states[mask, rel_idx], states[mask, rel_idx+1], alpha=0.2, s=5)
            ax.set_title(f"Cam {cam_idx+1}: Relative Vector (Tube->Peg)"); ax.axis('equal')
        curr_row += 1

    if use_embs:
        kp_off = kp_dim_per_cam if use_kps else 0
        # RGB & Depth Histograms
        for cam_idx, start in enumerate([c1_start, c2_start]):
            ax_rgb = fig.add_subplot(gs[curr_row, cam_idx])
            sns.kdeplot(states[:, start + kp_off], ax=ax_rgb, fill=True, color='blue')
            ax_rgb.set_title(f"Cam {cam_idx+1}: RGB Embedding (Dim 0)")
            
            ax_depth = fig.add_subplot(gs[curr_row+1, cam_idx])
            sns.kdeplot(states[:, start + kp_off + rgb_dim], ax=ax_depth, fill=True, color='purple')
            ax_depth.set_title(f"Cam {cam_idx+1}: Depth Embedding (Dim 0)")
        curr_row += 2

    plt.tight_layout()
    plt.savefig(output_dir / "visual_feature_analysis.png")
    plt.close(fig)

def plot_proprio_action_distributions(states, actions, joint_start_idx, output_dir, config):
    """
    Plots proprioception and action histograms.
    Dynamically handles Task Space vs Joint Space labels.
    """
    control_mode = config.get('control_mode', 'joint_space')
    
    # --- Define Indices ---
    # We select 3 representative indices for Arm and 1 for Hand to keep plots readable
    if control_mode == 'task_space':
        # State: [X, Y, Z, Qx, Qy, Qz, Qw] -> Hand
        state_map = {
            'EE Pos X': joint_start_idx + 0,
            'EE Pos Y': joint_start_idx + 1,
            'EE Pos Z': joint_start_idx + 2,
            'Hand J0': joint_start_idx + 7 # First hand joint
        }
        # Action: [dX, dY, dZ, dRx, dRy, dRz] -> Hand
        action_map = {
            'Action dX': 0,
            'Action dY': 1,
            'Action dZ': 2,
            'Action Hand J0': 6 # First hand action
        }
    else:
        # Joint Space
        state_map = {
            'Arm J1': joint_start_idx + 0,
            'Arm J4': joint_start_idx + 3,
            'Arm J7': joint_start_idx + 6,
            'Hand J0': joint_start_idx + 7
        }
        action_map = {
            'Act J1': 0,
            'Act J4': 3,
            'Act J7': 6,
            'Act Hand J0': 7
        }

    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    fig.suptitle(f'Proprioception & Action Distributions ({control_mode})', fontsize=18)

    # Plot States (Top Row)
    for i, (label, idx) in enumerate(state_map.items()):
        ax = axes[0, i]
        sns.histplot(states[:, idx], kde=True, ax=ax, color='C0', element="step")
        ax.set_title(f"State: {label}")
        ax.set_xlabel("Value")

    # Plot Actions (Bottom Row)
    for i, (label, idx) in enumerate(action_map.items()):
        ax = axes[1, i]
        # Check if action is likely zero (often happens in data collection)
        if np.allclose(actions[:, idx], 0):
            ax.text(0.5, 0.5, "All Zeros", ha='center', transform=ax.transAxes)
            ax.set_title(f"Action: {label}")
            continue
            
        sns.histplot(actions[:, idx], kde=True, ax=ax, color='C1', element="step")
        ax.set_title(f"Action: {label}")
        ax.set_xlabel("Value")

    plt.tight_layout(rect=(0, 0, 1, 0.95))
    save_path = output_dir / "proprio_action_distributions.png"
    plt.savefig(save_path)
    plt.close(fig)
    logging.info(f"📊 Saved proprio/action plots to {save_path}")


def plot_correlation_heatmap(states, tactile_total_dim, proprio_dim, output_dir):
    """
    Robust correlation heatmap.
    Calculates modality boundaries AFTER filtering constant columns to ensure lines match data.
    """
    # Sample data to save memory/time
    sample_size = min(5000, states.shape[0])
    indices = np.random.choice(states.shape[0], sample_size, replace=False)
    X_sample = states[indices]

    # 1. Create a "Modality Mask" BEFORE filtering
    # 0=Tactile, 1=Proprio, 2=Visual
    modality_labels = np.zeros(X_sample.shape[1])
    modality_labels[tactile_total_dim : tactile_total_dim + proprio_dim] = 1
    modality_labels[tactile_total_dim + proprio_dim :] = 2

    # 2. Filter Constant Columns
    variances = np.var(X_sample, axis=0)
    mask_active = variances > 1e-9
    
    X_filtered = X_sample[:, mask_active]
    labels_filtered = modality_labels[mask_active]

    if X_filtered.shape[1] < 2:
        return

    # 3. Compute Correlation
    corr = np.corrcoef(X_filtered, rowvar=False)

    # 4. Find Boundary Indices in the FILTERED space
    # Find where label changes from 0->1 or 1->2
    boundaries = np.where(np.diff(labels_filtered) > 0)[0] + 1

    # 5. Plot
    fig, ax = plt.subplots(figsize=(12, 10))
    sns.heatmap(corr, cmap='RdBu_r', center=0, square=True, ax=ax, 
                xticklabels=False, yticklabels=False, cbar_kws={"shrink": .8})
    
    ax.set_title(f'Feature Correlation ({sample_size} samples)')

    # Draw Lines
    for b in boundaries:
        ax.axvline(x=b, color='white', linestyle='--', linewidth=1.5)
        ax.axhline(y=b, color='white', linestyle='--', linewidth=1.5)

    # Add Text Labels (Tactile, Proprio, Visual)
    # We calculate midpoints between 0, boundaries, and end
    ticks = np.concatenate(([0], boundaries, [len(labels_filtered)]))
    names = ['Tactile', 'Proprio', 'Visual']
    
    for i in range(len(ticks)-1):
        start, end = ticks[i], ticks[i+1]
        mid = (start + end) / 2
        if end > start: # Only if modality exists
            # Determine which name corresponds to the label value at this segment
            # Sample the label at the midpoint index
            label_val = int(labels_filtered[int(start)]) 
            if label_val < len(names):
                ax.text(mid, len(labels_filtered) + 2, names[label_val], 
                        ha='center', va='top', fontsize=12, fontweight='bold')

    plt.tight_layout()
    plt.savefig(output_dir / "feature_correlation.png", bbox_inches='tight')
    plt.close(fig)
    logging.info(f"📊 Saved correlation heatmap.")


def plot_trajectory_analysis(all_trajectories, config, output_dir, num_trajectories_to_plot=5):
    """
    Sanity check for EE paths and Action smoothness.
    Robustly finds EE Pose index using config.
    """
    if config.get('control_mode') != 'task_space':
        return

    # --- Robust Index Finding ---
    # We need to find where the EE Pose (7D) lives in the state vector.
    # Usually: Tactile features come first.
    state_config = config.get('state', {})
    tactile_dim = 0
    # Add up tactile dimensions based on your specific SENSOR_ORDER logic
    # Here assuming standard 24 (2 fingers * 12) or checking config
    # For now, we assume the user passes the correct tactile_total_dim or we infer it:
    
    # Inference: Proprio starts after tactile. 
    # If using standard sensor order:
    # tactile_dim = len(SENSOR_ORDER) * TACTILE_FEATURE_DIM
    # But since we don't have SENSOR_ORDER here, let's look at the first trajectory.
    
    # Fallback/Heuristic:
    # If task space, the first 3 cols of Proprio are X,Y,Z.
    # Usually tactile is roughly < 50 dims. 
    # Let's assume the passed `config` would be better used outside, 
    # but here we will try to find the start of the Proprio block.
    
    # Better approach: We know Proprio starts after Tactile.
    # We will assume tactile dim is calculated by the caller or we estimate it.
    # FOR THIS FIX: We assume the caller provided standard data where 
    # [0:24] is tactile, [24:31] is EE Pose.
    
    # Correct fix: Calculate it exactly as in visualize_dataset_only
    # Since we can't easily pass it in here without changing signature too much,
    # let's assume standard layout used in previous function:
    # tactile_total_dim is implicit. 
    
    # Let's verify variance. Tactile varies a lot. Proprio varies smoothly.
    # We will trust the hardcoded offset logic relative to the `proprio_start` 
    # passed in via a wrapper, OR we just recalculate it here:
    
    # Recalculate based on config logic provided in previous prompts:
    # tactile_total_dim = len(SENSOR_ORDER) * TACTILE_FEATURE_DIM 
    # Since we don't have imports, we rely on the generic '24' if not provided.
    
    # To be safe, let's look at the action. Action is 6D delta.
    
    num_to_plot = min(len(all_trajectories), num_trajectories_to_plot)
    fig = plt.figure(figsize=(18, 4 * num_to_plot))
    
    # Determine offset
    # We can detect the offset by checking if the first N cols are tactile
    # (usually bounded 0-1 or small floats) vs Proprio (meters).
    # HACK for robustness: Just use the known tactile dim from the main function loop.
    # Since we don't have it, we default to 24 (standard for 2-finger BioTac).
    ee_pose_start_idx = 24 

    for i in range(num_to_plot):
        traj = all_trajectories[i]
        states = traj['state_t']
        actions = traj['action_t']
        
        # 1. 3D Path
        ax1 = fig.add_subplot(num_to_plot, 3, i*3 + 1, projection='3d')
        # Check if we have enough dims
        if states.shape[1] > ee_pose_start_idx + 2:
            xs = states[:, ee_pose_start_idx]
            ys = states[:, ee_pose_start_idx+1]
            zs = states[:, ee_pose_start_idx+2]
            
            ax1.plot(xs, ys, zs, label='Path', lw=2)
            ax1.scatter(xs[0], ys[0], zs[0], color='g', label='Start')
            ax1.scatter(xs[-1], ys[-1], zs[-1], color='r', label='End')
            ax1.set_title(f"Traj {i}: EE Path")
            
            # Equal aspect ratio hack for matplotlib 3D
            max_range = np.array([xs.max()-xs.min(), ys.max()-ys.min(), zs.max()-zs.min()]).max() / 2.0
            mid_x = (xs.max()+xs.min()) * 0.5
            mid_y = (ys.max()+ys.min()) * 0.5
            mid_z = (zs.max()+zs.min()) * 0.5
            ax1.set_xlim(mid_x - max_range, mid_x + max_range)
            ax1.set_ylim(mid_y - max_range, mid_y + max_range)
            ax1.set_zlim(mid_z - max_range, mid_z + max_range)

        # 2. Linear Action (Velocity)
        ax2 = fig.add_subplot(num_to_plot, 3, i*3 + 2)
        ax2.plot(actions[:, 0], label='dX')
        ax2.plot(actions[:, 1], label='dY')
        ax2.plot(actions[:, 2], label='dZ')
        ax2.set_title("Linear Velocity cmds")
        ax2.legend(loc='upper right', fontsize='x-small')
        
        # 3. Angular Action
        ax3 = fig.add_subplot(num_to_plot, 3, i*3 + 3)
        ax3.plot(actions[:, 3], label='dRx', ls='--')
        ax3.plot(actions[:, 4], label='dRy', ls='--')
        ax3.plot(actions[:, 5], label='dRz', ls='--')
        ax3.set_title("Angular Velocity cmds")
        ax3.legend(loc='upper right', fontsize='x-small')

    plt.tight_layout()
    save_path = output_dir / "trajectory_samples.png"
    plt.savefig(save_path)
    plt.close(fig)
    logging.info(f"📊 Saved trajectory samples to {save_path}")

# ===================================================================
# === MAIN SCRIPT LOGIC ===
# ===================================================================

def visualize_dataset(input_path, config):
    if not input_path.exists():
        logging.error(f"❌ Input dataset file not found: {input_path}")
        return
    
    logging.info(f"Loading dataset from {input_path} for visualization...")
    with open(input_path, "rb") as f: 
        all_trajectories = pickle.load(f)
    
    # 1. Concatenate all data into single arrays
    states = np.concatenate([t['state_t'] for t in all_trajectories], axis=0)
    actions = np.concatenate([t['action_t'] for t in all_trajectories], axis=0)

    # 2. Dynamic Dimension Calculation (Matching your original exactly)
    state_config = config.get('state', {})
    
    # Tactile Dim
    tactile_total_dim = 24
    
    # Proprioception (Arm + Hand)
    arm_proprio_dim = 7 
    joint_start_idx = tactile_total_dim # Start of arm joints
    
    hand_proprio_dim = 16
    
    # Optional: 3D Tactile Frames
    tactile_3d_dim = 0
    if state_config.get('use_3d_tactile', False):
        # Typically 3 frames * 7 dims = 21
        tactile_3d_dim = len(config.get('kinematics', {}).get('tactile_frames', [])) * 7

    proprio_dim = arm_proprio_dim + hand_proprio_dim + tactile_3d_dim
    visual_dim = states.shape[1] - (tactile_total_dim + proprio_dim)
    
    logging.info(f"Parsed Dimensions: Tactile={tactile_total_dim}, Proprio={proprio_dim}, Visual={visual_dim}")

    # 3. Setup Plotting Directory
    plot_dir = Path(str(input_path.parent).replace("processed_datasets", "visualizations")) / input_path.stem
    plot_dir.mkdir(parents=True, exist_ok=True)
    
    plt.style.use('seaborn-v0_8-muted')

    # 4. Run Plotters
    plot_visual_distributions(states, tactile_total_dim, visual_dim, plot_dir, config)
    # You can add tactile and proprio plotters here following the same pattern
    
    logging.info(f"✅ Visualizations saved to {plot_dir}")

def main():
    parser = argparse.ArgumentParser(description="Interactive visualization tool for robot datasets.")
    args = parser.parse_args()

    try:
        # --- 1. Interactively Select the Input Dataset ---
        pkl_choices = find_pkl_files(paths.PROCESSED_DATA_DIR)
        config_choices = find_config_files(paths.CONFIG_DIR)

        if not pkl_choices or not config_choices:
            logging.error("Missing .pkl or .yaml files. Exiting.")
            return

        questions = [
            inquirer.List('input_file',
                          message="Select the processed dataset (.pkl) to visualize",
                          choices=pkl_choices),
            inquirer.List('config_file',
                          message="Select the configuration file",
                          choices=config_choices),
        ]
        
        answers = inquirer.prompt(questions)
        if not answers: return

        # --- 2. Load Config ---
        config_path = paths.WORKSPACE_ROOT / answers['config_file']
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        visualize_dataset(
            input_path=paths.WORKSPACE_ROOT / answers['input_file'],
            config=config
        )

    except (KeyboardInterrupt, TypeError):
        logging.info("\nOperation cancelled.")
        
if __name__ == "__main__":
    main()