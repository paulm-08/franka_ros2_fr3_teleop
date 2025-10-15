import pickle
import numpy as np
import argparse
import logging
from pathlib import Path
import random
import matplotlib.pyplot as plt
import seaborn as sns

from model_pipeline import paths # Import the new paths module

# --- Logger Setup ---
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

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
    Generates detailed plots for a 16D visual feature vector (2 cameras, 2 objects, 4 features each).
    Feature order: [tube_x, tube_y, tube_conf, tube_flag, peg_x, peg_y, peg_conf, peg_flag] per camera.
    """
    if visual_dim != 16:
        logging.warning(f"This visualization expects a 16D visual vector, but found {visual_dim}D. Skipping visual plots.")
        return

    fig = plt.figure(figsize=(20, 24))
    gs = fig.add_gridspec(6, 2) # Added a row for the new detection rate plot
    fig.suptitle('Dual-Camera Keypoint Feature Analysis (with Detection Flags)', fontsize=18, y=1.01)

    # --- Feature Indices Setup (for 16D vector) ---
    c1_tube_x, c1_tube_y, c1_tube_conf, c1_tube_flag = tactile_total_dim, tactile_total_dim + 1, tactile_total_dim + 2, tactile_total_dim + 3
    c1_peg_x,  c1_peg_y,  c1_peg_conf,  c1_peg_flag  = tactile_total_dim + 4, tactile_total_dim + 5, tactile_total_dim + 6, tactile_total_dim + 7
    c2_tube_x, c2_tube_y, c2_tube_conf, c2_tube_flag = tactile_total_dim + 8, tactile_total_dim + 9, tactile_total_dim + 10, tactile_total_dim + 11
    c2_peg_x,  c2_peg_y,  c2_peg_conf,  c2_peg_flag  = tactile_total_dim + 12, tactile_total_dim + 13, tactile_total_dim + 14, tactile_total_dim + 15

    # --- NEW: Plot Row 1: Detection Rate Analysis ---
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

    # --- Plot Row 2: Coordinate Distributions ---
    ax1 = fig.add_subplot(gs[1, 0])
    sns.kdeplot(X_train[:, c1_tube_x], ax=ax1, label='Tube X', fill=True, warn_singular=False)
    sns.kdeplot(X_train[:, c1_peg_x], ax=ax1, label='Peg X', fill=True, warn_singular=False)
    ax1.set_title('Camera 1: X-Coordinate Distributions'); ax1.legend(); ax1.grid(True, linestyle='--')

    ax2 = fig.add_subplot(gs[1, 1])
    sns.kdeplot(X_train[:, c2_tube_x], ax=ax2, label='Tube X', fill=True, warn_singular=False)
    sns.kdeplot(X_train[:, c2_peg_x], ax=ax2, label='Peg X', fill=True, warn_singular=False)
    ax2.set_title('Camera 2: X-Coordinate Distributions'); ax2.legend(); ax2.grid(True, linestyle='--')

    # --- Plot Row 3: Confidence Score Distributions (only for detected frames) ---
    ax_conf1 = fig.add_subplot(gs[2, 0])
    ax_conf1.hist(X_train[X_train[:, c1_tube_flag] > 0, c1_tube_conf], bins=50, density=True, alpha=0.7, label='Tube Confidence')
    ax_conf1.hist(X_train[X_train[:, c1_peg_flag] > 0, c1_peg_conf], bins=50, density=True, alpha=0.7, label='Peg Confidence')
    ax_conf1.set_title('Camera 1: Detection Confidence (where detected)'); ax_conf1.legend(); ax_conf1.grid(True, linestyle='--')

    ax_conf2 = fig.add_subplot(gs[2, 1])
    ax_conf2.hist(X_train[X_train[:, c2_tube_flag] > 0, c2_tube_conf], bins=50, density=True, alpha=0.7, label='Tube Confidence')
    ax_conf2.hist(X_train[X_train[:, c2_peg_flag] > 0, c2_peg_conf], bins=50, density=True, alpha=0.7, label='Peg Confidence')
    ax_conf2.set_title('Camera 2: Detection Confidence (where detected)'); ax_conf2.legend(); ax_conf2.grid(True, linestyle='--')

   # --- Plot Row 4: Position Coverage (FILTERED) ---
    ax3 = fig.add_subplot(gs[3, 0])
    # --- MODIFICATION: Create a boolean mask for detected tube in camera 1 ---
    mask_train_c1_tube = X_train[:, c1_tube_flag] > 0
    ax3.scatter(X_train[mask_train_c1_tube, c1_tube_x], X_train[mask_train_c1_tube, c1_tube_y], alpha=0.05)
    ax3.set_title('Camera 1: Tube Position Coverage (Detected Only)')

    ax4 = fig.add_subplot(gs[3, 1])
    # --- MODIFICATION: Create a boolean mask for detected peg in camera 1 ---
    mask_train_c1_peg = X_train[:, c1_peg_flag] > 0
    ax4.scatter(X_train[mask_train_c1_peg, c1_peg_x], X_train[mask_train_c1_peg, c1_peg_y], alpha=0.05)
    ax4.set_title('Camera 1: Peg Position Coverage (Detected Only)')
    
    # --- Plot Row 5: Position Coverage for Camera 2 (FILTERED) ---
    ax5 = fig.add_subplot(gs[4, 0])
    # --- MODIFICATION: Create a boolean mask for detected tube in camera 2 ---
    mask_train_c2_tube = X_train[:, c2_tube_flag] > 0
    ax5.scatter(X_train[mask_train_c2_tube, c2_tube_x], X_train[mask_train_c2_tube, c2_tube_y], alpha=0.05)
    ax5.set_title('Camera 2: Tube Position Coverage (Detected Only)')

    ax6 = fig.add_subplot(gs[4, 1])
    # --- MODIFICATION: Create a boolean mask for detected peg in camera 2 ---
    mask_train_c2_peg = X_train[:, c2_peg_flag] > 0
    ax6.scatter(X_train[mask_train_c2_peg, c2_peg_x], X_train[mask_train_c2_peg, c2_peg_y], alpha=0.05)
    ax6.set_title('Camera 2: Peg Position Coverage (Detected Only)')

    # --- Plot Row 6: Relative Position Distributions (FILTERED) ---
    ax7 = fig.add_subplot(gs[5, 0])
    # --- MODIFICATION: Create a mask for frames where BOTH objects are seen in camera 1 ---
    mask_c1_both = (X_train[:, c1_tube_flag] > 0) & (X_train[:, c1_peg_flag] > 0)
    rel_x1 = X_train[mask_c1_both, c1_tube_x] - X_train[mask_c1_both, c1_peg_x]
    rel_y1 = X_train[mask_c1_both, c1_tube_y] - X_train[mask_c1_both, c1_peg_y]
    ax7.hist2d(rel_x1, rel_y1, bins=50, cmap='viridis')
    ax7.set_title('Camera 1: Relative Position (Where Both Detected)')
    
    ax8 = fig.add_subplot(gs[5, 1])
    # --- MODIFICATION: Create a mask for frames where BOTH objects are seen in camera 2 ---
    mask_c2_both = (X_train[:, c2_tube_flag] > 0) & (X_train[:, c2_peg_flag] > 0)
    rel_x2 = X_train[mask_c2_both, c2_tube_x] - X_train[mask_c2_both, c2_peg_x]
    rel_y2 = X_train[mask_c2_both, c2_tube_y] - X_train[mask_c2_both, c2_peg_y]
    ax8.hist2d(rel_x2, rel_y2, bins=50, cmap='viridis')
    ax8.set_title('Camera 2: Relative Position (Where Both Detected)')

    for ax in [ax3, ax4, ax5, ax6, ax7, ax8]:
        ax.set_xlabel('X (normalized)'); ax.set_ylabel('Y (normalized)')
        ax.grid(True, linestyle='--'); ax.axis('equal')
        ax.set_xlim(0, 1); ax.set_ylim(0, 1) # Set limits for normalized coordinates

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
    parser = argparse.ArgumentParser(description="Analyze, visualize, and optionally split a trajectory dataset.")
    # Use dynamic paths as defaults
    parser.add_argument("--input_file", type=str, default=str(paths.PROCESSED_DATA_DIR / "dataset.pkl"), help="Path to the input .pkl dataset file.")
    parser.add_argument("--output_file", type=str, default=str(paths.PROCESSED_DATA_DIR / "dataset_split.npz"), help="Optional: Path to save the final split .npz file.")
    parser.add_argument("--split_ratio", type=float, default=0.85, help="Fraction of trajectories for training.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for shuffling.")
    args = parser.parse_args()

    visualize_and_split_dataset(
        input_path=Path(args.input_file),
        output_path=Path(args.output_file) if args.output_file else None,
        split_ratio=args.split_ratio,
        seed=args.seed
    )

if __name__ == "__main__":
    main()

