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

from model_pipeline import paths # Import the new paths module

# --- Logger Setup ---
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def find_pkl_files(search_path):
    """Finds all .pkl files in the specified directory."""
    logging.info(f"Searching for processed datasets (.pkl) in: {search_path}...")
    found_files = [p.relative_to(paths.WORKSPACE_ROOT) for p in search_path.glob("*.pkl")]
    logging.info(f"Found {len(found_files)} files.")
    return [str(p) for p in found_files]

# ===================================================================
# === VISUALIZATION HELPER FUNCTIONS ===
# ===================================================================

def plot_tactile_distributions(X_train, X_val, output_dir):
    """Generates a detailed plot for tactile feature distributions."""
    tactile_feature_names = [
        'centroid_x', 'centroid_y', 'major_axis_x', 'major_axis_y',
        'shape_len (eig_maj)', 'shape_width (eig_min)', 'total_force', 'contact_flag'
    ]
    fig, axes = plt.subplots(4, 2, figsize=(15, 20))
    fig.suptitle('Detailed Tactile Feature Distributions (Index Finger)', fontsize=18, y=1.02)
    axes = axes.flatten()

    for i, feature_name in enumerate(tactile_feature_names):
        ax = axes[i]
        feature_idx = i
        
        # Filter non-contact data for shape/orientation plots to make them more readable
        if 'centroid' in feature_name or 'axis' in feature_name or 'shape' in feature_name:
            train_feature = X_train[X_train[:, 7] > 0, feature_idx]
            val_feature = X_val[X_val[:, 7] > 0, feature_idx]
            ax.set_title(f'"{feature_name}" (Contact Only)')
        else:
            train_feature = X_train[:, feature_idx]
            val_feature = X_val[:, feature_idx]
            ax.set_title(f'"{feature_name}"')

        ax.hist(train_feature, bins=50, density=True, alpha=0.7, label='Train Set')
        ax.hist(val_feature, bins=50, density=True, alpha=0.7, label='Validation Set')
        ax.set_xlabel('Feature Value'); ax.set_ylabel('Density')
        ax.legend(); ax.grid(True, linestyle='--')

    plt.tight_layout(rect=[0, 0, 1, 1])
    save_path = output_dir / "tactile_feature_distributions.png"
    plt.savefig(save_path)
    plt.close(fig)
    logging.info(f"📊 Saved detailed tactile plot to {save_path}")

def plot_visual_distributions(X_train, X_val, tactile_total_dim, visual_dim, output_dir):
    """
    Generates detailed plots for the 788D "kitchen sink" visual feature vector.
    Structure per camera: [Keypoints(10D), RGB_Emb(256D), Depth_Emb(128D)]
    """
    # --- Define the dimensions of each sub-feature ---
    kp_dim = 10  # (x, y, conf, flag) * 2 + (rel_x, rel_y)
    rgb_dim = 32
    depth_dim = 16
    
    single_cam_dim = kp_dim + rgb_dim + depth_dim
    expected_visual_dim = single_cam_dim * 2 # 394 * 2 = 788

    if visual_dim != expected_visual_dim:
        logging.warning(f"This visualization expects a {expected_visual_dim}D visual vector, but found {visual_dim}D. Skipping visual plots.")

    fig = plt.figure(figsize=(20, 30))
    gs = fig.add_gridspec(7, 2) # Now 7 rows
    fig.suptitle('Hybrid Visual Feature Analysis (Keypoints + Embeddings)', fontsize=18, y=1.01)

    # --- Feature Indices Setup (for 788D vector) ---
    # Camera 1 (starts at tactile_total_dim)
    c1_start = tactile_total_dim
    c1_tube_x, c1_tube_y, c1_tube_conf, c1_tube_flag = c1_start, c1_start + 1, c1_start + 2, c1_start + 3
    c1_peg_x,  c1_peg_y,  c1_peg_conf,  c1_peg_flag  = c1_start + 4, c1_start + 5, c1_start + 6, c1_start + 7
    c1_rel_x,  c1_rel_y                             = c1_start + 8, c1_start + 9
    c1_rgb_emb_start = c1_start + kp_dim
    c1_depth_emb_start = c1_rgb_emb_start + rgb_dim
    
    # Camera 2 (starts after camera 1)
    c2_start = tactile_total_dim + single_cam_dim
    c2_tube_x, c2_tube_y, c2_tube_conf, c2_tube_flag = c2_start, c2_start + 1, c2_start + 2, c2_start + 3
    c2_peg_x,  c2_peg_y,  c2_peg_conf,  c2_peg_flag  = c2_start + 4, c2_start + 5, c2_start + 6, c2_start + 7
    c2_rel_x,  c2_rel_y                             = c2_start + 8, c2_start + 9
    c2_rgb_emb_start = c2_start + kp_dim
    c2_depth_emb_start = c2_rgb_emb_start + rgb_dim

    # --- Plot Row 1: Detection Rate Analysis ---
    ax_rate1 = fig.add_subplot(gs[0, 0])
    tube1_rate = np.mean(X_train[:, c1_tube_flag]) * 100
    peg1_rate = np.mean(X_train[:, c1_peg_flag]) * 100
    ax_rate1.bar(['Tube Tip', 'Peg'], [tube1_rate, peg1_rate], color=['skyblue', 'salmon'])
    ax_rate1.set_title('Camera 1: Object Detection Rate (%)', fontsize=14, weight='bold')
    ax_rate1.set_ylabel('Frames Detected (%)'); ax_rate1.set_ylim(0, 105)
    for i, rate in enumerate([tube1_rate, peg1_rate]):
        ax_rate1.text(i, rate + 2, f'{rate:.1f}%', ha='center', fontsize=12)
    ax_rate1.grid(axis='y', linestyle='--')
    
    ax_rate2 = fig.add_subplot(gs[0, 1])
    tube2_rate = np.mean(X_train[:, c2_tube_flag]) * 100
    peg2_rate = np.mean(X_train[:, c2_peg_flag]) * 100
    ax_rate2.bar(['Tube Tip', 'Peg'], [tube2_rate, peg2_rate], color=['skyblue', 'salmon'])
    ax_rate2.set_title('Camera 2: Object Detection Rate (%)', fontsize=14, weight='bold')
    ax_rate2.set_ylim(0, 105)
    for i, rate in enumerate([tube2_rate, peg2_rate]):
        ax_rate2.text(i, rate + 2, f'{rate:.1f}%', ha='center', fontsize=12)
    ax_rate2.grid(axis='y', linestyle='--')

    # --- Plot Row 2: Confidence Score Distributions ---
    ax_conf1 = fig.add_subplot(gs[1, 0])
    ax_conf1.hist(X_train[X_train[:, c1_tube_flag] > 0, c1_tube_conf], bins=50, density=True, alpha=0.7, label='Tube Confidence')
    ax_conf1.hist(X_train[X_train[:, c1_peg_flag] > 0, c1_peg_conf], bins=50, density=True, alpha=0.7, label='Peg Confidence')
    ax_conf1.set_title('Camera 1: Detection Confidence (where detected)'); ax_conf1.legend(); ax_conf1.grid(True, linestyle='--')

    ax_conf2 = fig.add_subplot(gs[1, 1])
    ax_conf2.hist(X_train[X_train[:, c2_tube_flag] > 0, c2_tube_conf], bins=50, density=True, alpha=0.7, label='Tube Confidence')
    ax_conf2.hist(X_train[X_train[:, c2_peg_flag] > 0, c2_peg_conf], bins=50, density=True, alpha=0.7, label='Peg Confidence')
    ax_conf2.set_title('Camera 2: Detection Confidence (where detected)'); ax_conf2.legend(); ax_conf2.grid(True, linestyle='--')
    
    # --- Plot Row 3: COMBINED Position Coverage ---
    ax_pos1 = fig.add_subplot(gs[2, 0])
    mask_c1_tube = X_train[:, c1_tube_flag] > 0
    mask_c1_peg = X_train[:, c1_peg_flag] > 0
    ax_pos1.scatter(X_train[mask_c1_tube, c1_tube_x], X_train[mask_c1_tube, c1_tube_y], alpha=0.05, color='blue')
    ax_pos1.scatter(X_train[mask_c1_peg, c1_peg_x], X_train[mask_c1_peg, c1_peg_y], alpha=0.05, color='red')
    ax_pos1.set_title('Camera 1: Position Coverage (Detected Only)')
    legend_elements_pos = [Line2D([0], [0], marker='o', color='w', label='Tube Tip', markerfacecolor='blue', markersize=10),
                           Line2D([0], [0], marker='o', color='w', label='Peg', markerfacecolor='red', markersize=10)]
    ax_pos1.legend(handles=legend_elements_pos)

    ax_pos2 = fig.add_subplot(gs[2, 1])
    mask_c2_tube = X_train[:, c2_tube_flag] > 0
    mask_c2_peg = X_train[:, c2_peg_flag] > 0
    ax_pos2.scatter(X_train[mask_c2_tube, c2_tube_x], X_train[mask_c2_tube, c2_tube_y], alpha=0.05, color='blue')
    ax_pos2.scatter(X_train[mask_c2_peg, c2_peg_x], X_train[mask_c2_peg, c2_peg_y], alpha=0.05, color='red')
    ax_pos2.set_title('Camera 2: Position Coverage (Detected Only)')
    ax_pos2.legend(handles=legend_elements_pos)

    # --- Plot Row 4: Engineered Relative Position Distributions ---
    ax_rel_dist1 = fig.add_subplot(gs[3, 0])
    mask_c1_both = (X_train[:, c1_tube_flag] > 0) & (X_train[:, c1_peg_flag] > 0)
    sns.kdeplot(X_train[mask_c1_both, c1_rel_x], ax=ax_rel_dist1, label='Relative X', fill=True, warn_singular=False)
    sns.kdeplot(X_train[mask_c1_both, c1_rel_y], ax=ax_rel_dist1, label='Relative Y', fill=True, warn_singular=False)
    ax_rel_dist1.set_title(f'Camera 1: Engineered Relative Coords ({np.sum(mask_c1_both)} frames)')
    ax_rel_dist1.legend(); ax_rel_dist1.grid(True, linestyle='--')

    ax_rel_dist2 = fig.add_subplot(gs[3, 1])
    mask_c2_both = (X_train[:, c2_tube_flag] > 0) & (X_train[:, c2_peg_flag] > 0)
    sns.kdeplot(X_train[mask_c2_both, c2_rel_x], ax=ax_rel_dist2, label='Relative X', fill=True, warn_singular=False)
    sns.kdeplot(X_train[mask_c2_both, c2_rel_y], ax=ax_rel_dist2, label='Relative Y', fill=True, warn_singular=False)
    ax_rel_dist2.set_title(f'Camera 2: Engineered Relative Coords ({np.sum(mask_c2_both)} frames)')
    ax_rel_dist2.legend(); ax_rel_dist2.grid(True, linestyle='--')
    
    # --- Plot Row 5: Calculated Relative Position (for Verification) ---
    ax_rel_scatter1 = fig.add_subplot(gs[4, 0])
    rel_x1_calc = X_train[mask_c1_both, c1_tube_x] - X_train[mask_c1_both, c1_peg_x]
    rel_y1_calc = X_train[mask_c1_both, c1_tube_y] - X_train[mask_c1_both, c1_peg_y]
    ax_rel_scatter1.scatter(rel_x1_calc, rel_y1_calc, alpha=0.1)
    ax_rel_scatter1.set_title('Camera 1: Calculated Relative Position')

    ax_rel_scatter2 = fig.add_subplot(gs[4, 1])
    rel_x2_calc = X_train[mask_c2_both, c2_tube_x] - X_train[mask_c2_both, c2_peg_x]
    rel_y2_calc = X_train[mask_c2_both, c2_tube_y] - X_train[mask_c2_both, c2_peg_y]
    ax_rel_scatter2.scatter(rel_x2_calc, rel_y2_calc, alpha=0.1)
    ax_rel_scatter2.set_title('Camera 2: Calculated Relative Position')

    # --- NEW: Plot Row 6: RGB Embedding Distributions ---
    ax_emb_rgb1 = fig.add_subplot(gs[5, 0])
    sns.kdeplot(X_train[:, c1_rgb_emb_start + 10], ax=ax_emb_rgb1, label='Train Emb[10]', fill=True)
    sns.kdeplot(X_val[:, c1_rgb_emb_start + 10], ax=ax_emb_rgb1, label='Val Emb[10]', fill=True, linestyle='--')
    sns.kdeplot(X_train[:, c1_rgb_emb_start + 50], ax=ax_emb_rgb1, label='Train Emb[50]', fill=True)
    ax_emb_rgb1.set_title('Camera 1: Sample RGB Embedding Dists'); ax_emb_rgb1.legend()
    
    ax_emb_rgb2 = fig.add_subplot(gs[5, 1])
    sns.kdeplot(X_train[:, c2_rgb_emb_start + 10], ax=ax_emb_rgb2, label='Train Emb[10]', fill=True)
    sns.kdeplot(X_val[:, c2_rgb_emb_start + 10], ax=ax_emb_rgb2, label='Val Emb[10]', fill=True, linestyle='--')
    sns.kdeplot(X_train[:, c2_rgb_emb_start + 50], ax=ax_emb_rgb2, label='Train Emb[50]', fill=True)
    ax_emb_rgb2.set_title('Camera 2: Sample RGB Embedding Dists'); ax_emb_rgb2.legend()
    
    # --- NEW: Plot Row 7: Depth Embedding Distributions ---
    ax_emb_d1 = fig.add_subplot(gs[6, 0])
    sns.kdeplot(X_train[:, c1_depth_emb_start + 10], ax=ax_emb_d1, label='Train Emb[10]', fill=True)
    sns.kdeplot(X_val[:, c1_depth_emb_start + 10], ax=ax_emb_d1, label='Val Emb[10]', fill=True, linestyle='--')
    # sns.kdeplot(X_train[:, c1_depth_emb_start + 50], ax=ax_emb_d1, label='Train Emb[50]', fill=True)
    ax_emb_d1.set_title('Camera 1: Sample Depth Embedding Dists'); ax_emb_d1.legend()
    
    ax_emb_d2 = fig.add_subplot(gs[6, 1])
    sns.kdeplot(X_train[:, c2_depth_emb_start + 10], ax=ax_emb_d2, label='Train Emb[10]', fill=True)
    sns.kdeplot(X_val[:, c2_depth_emb_start + 10], ax=ax_emb_d2, label='Val Emb[10]', fill=True, linestyle='--')
    # sns.kdeplot(X_train[:, c2_depth_emb_start + 50], ax=ax_emb_d2, label='Train Emb[50]', fill=True)
    ax_emb_d2.set_title('Camera 2: Sample Depth Embedding Dists'); ax_emb_d2.legend()

    # --- Styling ---
    for ax in [ax_pos1, ax_pos2]:
        ax.set_xlabel('X (normalized)'); ax.set_ylabel('Y (normalized)')
        ax.grid(True, linestyle='--'); ax.axis('equal')
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)

    for ax in [ax_rel_scatter1, ax_rel_scatter2]:
        ax.set_xlabel('Relative X'); ax.set_ylabel('Relative Y')
        ax.grid(True, linestyle='--'); ax.axis('equal')
        
    for ax in [ax_rel_dist1, ax_rel_dist2, ax_emb_rgb1, ax_emb_rgb2, ax_emb_d1, ax_emb_d2]:
        ax.grid(True, linestyle='--')
        ax.set_xlabel('Feature Value'); ax.set_ylabel('Density')

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    save_path = output_dir / "visual_feature_analysis.png"
    plt.savefig(save_path)
    plt.close(fig)
    logging.info(f"📊 Saved detailed visual plot to {save_path}")

def plot_proprio_action_distributions(X_train, y_train, X_val, y_val, joint_start_idx, output_dir):
    """Generates plots for proprioceptive state and action distributions."""
    joint_indices_to_plot = {
        'arm_joint_0': joint_start_idx,
        'arm_joint_4 (elbow)': joint_start_idx + 4,
        'hand_finger_joint_1': joint_start_idx + 20,
        'hand_finger_joint_2': joint_start_idx + 21,
    }
    action_indices_to_plot = {
        'action_joint_0': 0,
        'action_joint_4 (elbow)': 4,
        'action_finger_1': 20,
        'action_finger_2': 21,
    }

    fig, axes = plt.subplots(2, len(joint_indices_to_plot), figsize=(20, 10))
    fig.suptitle('Proprioceptive State and Action Distributions', fontsize=18, y=1.02)

    for i, (name, idx) in enumerate(joint_indices_to_plot.items()):
        ax = axes[0, i]
        ax.hist(X_train[:, idx], bins=50, density=True, alpha=0.7, label='Train Set')
        ax.hist(X_val[:, idx], bins=50, density=True, alpha=0.7, label='Validation Set')
        ax.set_title(f'State: {name}'); ax.legend(); ax.grid(True, linestyle='--')

    for i, (name, idx) in enumerate(action_indices_to_plot.items()):
        ax = axes[1, i]
        ax.hist(y_train[:, idx], bins=50, density=True, alpha=0.7, label='Train Set')
        ax.hist(y_val[:, idx], bins=50, density=True, alpha=0.7, label='Validation Set')
        ax.set_title(f'Action: {name}'); ax.legend(); ax.grid(True, linestyle='--')
        
    plt.tight_layout(rect=[0, 0, 1, 1])
    save_path = output_dir / "proprio_action_distributions.png"
    plt.savefig(save_path)
    plt.close(fig)
    logging.info(f"📊 Saved proprioception and action plot to {save_path}")

def plot_correlation_heatmap(X_train, tactile_total_dim, visual_dim, output_dir):
    """
    Generates and saves a feature correlation heatmap, dynamically drawing lines
    to separate feature modalities even after removing constant columns.
    """
    sample_size = min(5000, X_train.shape[0])
    sample_indices = np.random.choice(X_train.shape[0], sample_size, replace=False)
    X_sample = X_train[sample_indices]
    
    # --- Step 1: Identify and filter out constant features ---
    variances = np.var(X_sample, axis=0)
    non_constant_cols_mask = variances > 1e-9
    X_filtered = X_sample[:, non_constant_cols_mask]
    
    # Check if there's anything left to plot
    if X_filtered.shape[1] < 2:
        logging.warning("Not enough varying features to generate a correlation heatmap.")
        return

    correlation_matrix = np.corrcoef(X_filtered, rowvar=False)

    # --- Step 2: Calculate the NEW boundary indices after filtering ---
    # This is the key fix: we count how many features from each modality *remained*.
    
    # Original boundaries
    tactile_end_orig = tactile_total_dim
    visual_end_orig = tactile_total_dim + visual_dim

    # Count how many non-constant columns fall within each original modality block
    new_tactile_boundary = np.sum(non_constant_cols_mask[:tactile_end_orig])
    new_visual_boundary = new_tactile_boundary + np.sum(non_constant_cols_mask[tactile_end_orig:visual_end_orig])
    
    # --- Step 3: Plot the Heatmap and Lines ---
    fig, ax = plt.subplots(figsize=(14, 12))
    sns.heatmap(correlation_matrix, cmap='viridis', ax=ax, xticklabels=False, yticklabels=False)
    ax.set_title('Feature Correlation Matrix (Sample from Training Set)')
    ax.set_xlabel('Feature Index (after filtering constant columns)')
    ax.set_ylabel('Feature Index (after filtering constant columns)')
    
    # Draw vertical and horizontal lines at the new boundaries
    line_props = {'color': 'white', 'linestyle': '--', 'linewidth': 2}
    ax.axvline(x=new_tactile_boundary, **line_props)
    ax.axhline(y=new_tactile_boundary, **line_props)
    ax.axvline(x=new_visual_boundary, **line_props)
    ax.axhline(y=new_visual_boundary, **line_props)
    
    # Add text labels positioned in the middle of each new block
    # We add a small check to ensure the block still exists (has width > 0)
    if new_tactile_boundary > 0:
        ax.text(new_tactile_boundary / 2, -10, 'Tactile', ha='center', color='white', weight='bold')
    if new_visual_boundary - new_tactile_boundary > 0:
        ax.text(new_tactile_boundary + (new_visual_boundary - new_tactile_boundary) / 2, -10, 'Visual', ha='center', color='white', weight='bold')
    if X_filtered.shape[1] - new_visual_boundary > 0:
        ax.text(new_visual_boundary + (X_filtered.shape[1] - new_visual_boundary) / 2, -10, 'Proprio', ha='center', color='white', weight='bold')

    plt.tight_layout()
    save_path = output_dir / "feature_correlation_heatmap.png"
    plt.savefig(save_path)
    plt.close(fig)
    logging.info(f"📊 Saved feature correlation heatmap to {save_path}")
    
# ===================================================================
# === MAIN SCRIPT LOGIC ===
# ===================================================================

def visualize_and_split_dataset(input_path, output_path, split_ratio=0.8, seed=42):
    if not input_path.exists():
        logging.error(f"❌ Input dataset file not found: {input_path}")
        return
    
    # --- 1. Load and Split Trajectories ---
    logging.info(f"Loading trajectories from {input_path}...")
    with open(input_path, "rb") as f: all_trajectories = pickle.load(f)
    
    random.seed(seed); random.shuffle(all_trajectories)
    split_index = int(len(all_trajectories) * split_ratio)
    train_trajectories, val_trajectories = all_trajectories[:split_index], all_trajectories[split_index:]
    logging.info(f"Splitting into {len(train_trajectories)} train and {len(val_trajectories)} val trajectories.")

    # --- 2. Concatenate Data for Analysis ---
    def concatenate_trajectories(trajectories):
        if not trajectories: return (np.array([]),)*4 # Return empty arrays of correct tuple length
        tactile = np.concatenate([t['tactile_t'] for t in trajectories], axis=0)
        visual = np.concatenate([t['visual_t'] for t in trajectories], axis=0)
        joints = np.concatenate([t['joints_t'] for t in trajectories], axis=0)
        actions = np.concatenate([t['delta_q'] for t in trajectories], axis=0)
        return tactile, visual, joints, actions

    tactile_train, visual_train, joints_train, y_train = concatenate_trajectories(train_trajectories)
    tactile_val, visual_val, joints_val, y_val = concatenate_trajectories(val_trajectories)
    
    X_train = np.concatenate([tactile_train, visual_train, joints_train], axis=1)
    X_val = np.concatenate([tactile_val, visual_val, joints_val], axis=1)

    # --- 3. Dynamic Dimension Calculation for Plotting ---
    tactile_total_dim = tactile_train.shape[1]
    visual_dim = visual_train.shape[1]
    joint_start_idx = tactile_total_dim + visual_dim
    logging.info(f"Data dimensions: Tactile={tactile_total_dim}, Visual={visual_dim}, Proprio={joints_train.shape[1]}")

    # --- 4. Generate All Visualizations ---
    plot_dir = Path(output_path).parent if output_path else Path("./data/visualizations")
    plot_dir.mkdir(parents=True, exist_ok=True)
    logging.info(f"Generating visualizations in {plot_dir}...")
    
    if X_val.shape[0] > 0:
        plot_tactile_distributions(X_train, X_val, plot_dir)
        plot_visual_distributions(X_train, X_val, tactile_total_dim, visual_dim, plot_dir)
        plot_proprio_action_distributions(X_train, y_train, X_val, y_val, joint_start_idx, plot_dir)
        plot_correlation_heatmap(X_train, tactile_total_dim, visual_dim, plot_dir)
    
    # --- 5. Save Split Dataset (Optional) ---
    if output_path is None:
        logging.info("Visualization complete. No output file specified, data was not saved.")
    else:
        logging.info(f"Saving split dataset to {output_path}...")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(output_path, X_train=X_train, y_train=y_train, X_val=X_val, y_val=y_val)
        logging.info(f"✅ Split dataset saved to {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Interactively analyze, visualize, and split a trajectory dataset.")
    # The script is now fully interactive, so no arguments are required.
    args = parser.parse_args()

    try:
        # --- 1. Interactively Select the Input Dataset ---
        pkl_choices = find_pkl_files(paths.PROCESSED_DATA_DIR)
        if not pkl_choices:
            logging.error(f"No processed dataset (.pkl) files found in {paths.PROCESSED_DATA_DIR}. Please run dataset_builder.py first.")
            return

        questions = [
            inquirer.List('input_file',
                          message="Select the processed dataset (.pkl) to analyze and split",
                          choices=pkl_choices),
            inquirer.Text('output_filename',
                          message="Enter the name for the output split file (or leave blank to only visualize)",
                          default="dataset_split.npz"),
            inquirer.Text('split_ratio',
                          message="Enter the training split ratio (e.g., 0.85 for 85%)",
                          default="0.85"),
        ]
        
        answers = inquirer.prompt(questions)
        if not answers:
            logging.info("No selection made. Exiting.")
            return

        # --- 2. Process User Selections ---
        input_path = paths.WORKSPACE_ROOT / answers['input_file']
        split_ratio = float(answers['split_ratio'])
        
        output_filename = answers['output_filename']
        if output_filename and output_filename.strip():
            output_path = paths.PROCESSED_DATA_DIR / output_filename.strip()
        else:
            output_path = None # Visualization-only mode

    except (KeyboardInterrupt, TypeError):
        logging.info("\nOperation cancelled by user.")
        return
    except ValueError:
        logging.error("Invalid split ratio. Please enter a number between 0 and 1.")
        return

    # --- 3. Run the Main Logic ---
    visualize_and_split_dataset(
        input_path=input_path,
        output_path=output_path,
        split_ratio=split_ratio,
        seed=42 # Using a fixed seed for consistency
    )

if __name__ == "__main__":
    main()