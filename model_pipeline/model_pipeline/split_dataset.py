import pickle
import numpy as np
import argparse
import logging
from pathlib import Path
import random
import matplotlib.pyplot as plt
import seaborn as sns

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
    """Generates detailed plots for visual features, including YOLO and embeddings."""
    # --- Define the structure of your visual vector ---
    yolo_dim = 4
    yolo_feature_names = ['tube_x', 'tube_y', 'peg_x', 'peg_y']
    
    # Check if there are embeddings in addition to YOLO features
    has_embeddings = visual_dim > yolo_dim
    
    fig = plt.figure(figsize=(20, 15))
    gs = fig.add_gridspec(3, 2)
    fig.suptitle('Detailed Visual Feature Analysis', fontsize=18, y=1.02)

    # Plot 1: YOLO Coordinate Distributions
    ax1 = fig.add_subplot(gs[0, :])
    for i in range(yolo_dim):
        feature_idx = tactile_total_dim + i
        # Use a simple histogram which is robust to zero-variance data
        ax1.hist(X_train[:, feature_idx], bins=50, density=True, alpha=0.6, label=f'Train {yolo_feature_names[i]}')
    ax1.set_title('YOLO Object Coordinate Distributions (Training Set)')
    ax1.legend(); ax1.grid(True, linestyle='--')
    
    # --- The rest of the plots are the same as before ---
    # Plot 2 & 3: 2D Position Heatmaps
    ax2 = fig.add_subplot(gs[1, 0])
    tube_x_idx, tube_y_idx = tactile_total_dim, tactile_total_dim + 1
    ax2.scatter(X_train[:, tube_x_idx], X_train[:, tube_y_idx], alpha=0.05, label="Train")
    ax2.scatter(X_val[:, tube_x_idx], X_val[:, tube_y_idx], alpha=0.05, label="Val")
    ax2.set_title('Tube Position Coverage'); ax2.set_xlabel('X Coordinate'); ax2.set_ylabel('Y Coordinate')
    ax2.legend(); ax2.grid(True, linestyle='--'); ax2.axis('equal')

    ax3 = fig.add_subplot(gs[1, 1])
    peg_x_idx, peg_y_idx = tactile_total_dim + 2, tactile_total_dim + 3
    ax3.scatter(X_train[:, peg_x_idx], X_train[:, peg_y_idx], alpha=0.05)
    ax3.set_title('Peg Position Coverage'); ax3.set_xlabel('X Coordinate'); ax3.set_ylabel('Y Coordinate')
    ax3.grid(True, linestyle='--'); ax3.axis('equal')

    # Plot 4: Relative Position Distribution
    ax4 = fig.add_subplot(gs[2, 0])
    rel_x_train = X_train[:, tube_x_idx] - X_train[:, peg_x_idx]
    rel_y_train = X_train[:, tube_y_idx] - X_train[:, peg_y_idx]
    ax4.hist2d(rel_x_train, rel_y_train, bins=50, cmap='viridis')
    ax4.set_title('Relative Position (Tube - Peg) Distribution'); ax4.set_xlabel('Relative X'); ax4.set_ylabel('Relative Y')
    ax4.grid(True, linestyle='--'); ax4.axis('equal')
    
    # --- FIX: Only plot embeddings if they exist ---
    ax5 = fig.add_subplot(gs[2, 1])
    if has_embeddings:
        emb_start_idx = tactile_total_dim + yolo_dim
        # Check to ensure indices are valid before plotting
        if emb_start_idx + 50 < X_train.shape[1]:
            sns.kdeplot(X_train[:, emb_start_idx + 10], ax=ax5, label='Train Emb[10]', fill=True)
            sns.kdeplot(X_val[:, emb_start_idx + 10], ax=ax5, label='Val Emb[10]', fill=True, linestyle='--')
            sns.kdeplot(X_train[:, emb_start_idx + 50], ax=ax5, label='Train Emb[50]', fill=True)
            sns.kdeplot(X_val[:, emb_start_idx + 50], ax=ax5, label='Val Emb[50]', fill=True, linestyle='--')
            ax5.set_title('Sample of Abstract Embedding Distributions')
        else:
            ax5.text(0.5, 0.5, 'Not enough embedding features to plot.', ha='center', va='center')
    else:
        ax5.text(0.5, 0.5, 'No abstract embeddings found in data.', ha='center', va='center')
        ax5.set_title('Abstract Embedding Distributions')
    
    ax5.legend(); ax5.grid(True, linestyle='--')

    plt.tight_layout(rect=[0, 0, 1, 1])
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
    parser.add_argument("--input_file", type=str, required=True, help="Path to the input .pkl dataset file.")
    parser.add_argument("--output_file", type=str, help="Optional: Path to save the final split .npz file.")
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

