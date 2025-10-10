import pickle
import numpy as np
import argparse
import logging
from pathlib import Path
import random
import matplotlib.pyplot as plt
import seaborn as sns

# --- Logger Setup ---
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()]
)

def visualize_split_distributions(X_train, y_train, X_val, y_val, output_dir):
    """
    Generates and saves a comprehensive set of plots to visualize and validate
    the train/validation data split across all modalities.
    """
    logging.info("Generating detailed data visualization plots...")
    output_dir.mkdir(parents=True, exist_ok=True)

    # --- Dynamically calculate feature dimensions ---
    num_tactile_sensors = 3
    tactile_feature_dim = 8
    tactile_total_dim = num_tactile_sensors * tactile_feature_dim
    proprio_dim = y_train.shape[1]
    visual_dim = X_train.shape[1] - tactile_total_dim - proprio_dim
    joint_start_idx = tactile_total_dim + visual_dim
    
    logging.info(f"Plotting with dimensions: Tactile={tactile_total_dim}, Visual={visual_dim}, Proprio={proprio_dim}")

    # ===================================================================
    # === PLOT 1: Detailed Tactile Feature Distributions (Index Finger) ===
    # ===================================================================
    tactile_feature_names = [
        'centroid_x', 'centroid_y', 'major_axis_x', 'major_axis_y',
        'shape_len (eig_maj)', 'shape_width (eig_min)', 'total_force', 'contact_flag'
    ]
    
    fig1, axes = plt.subplots(4, 2, figsize=(15, 20))
    fig1.suptitle('Detailed Tactile Feature Distributions (Index Finger)', fontsize=18, y=1.03)
    axes = axes.flatten()

    for i, feature_name in enumerate(tactile_feature_names):
        ax = axes[i]
        feature_idx = i # We are looking at the first sensor (index finger)
        
        # Filter out non-contact data for shape/orientation plots to make them more readable
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
    save_path1 = output_dir / "tactile_feature_distributions.png"
    plt.savefig(save_path1)
    plt.close(fig1)
    logging.info(f"📊 Saved detailed tactile plot to {save_path1}")

    # =================================================================
    # === PLOT 2: Proprioceptive (Joint) & Action Distributions ===
    # =================================================================
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

    fig2, axes = plt.subplots(2, len(joint_indices_to_plot), figsize=(20, 10))
    fig2.suptitle('Proprioceptive State and Action Distributions', fontsize=18, y=1.03)

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
    save_path2 = output_dir / "proprio_action_distributions.png"
    plt.savefig(save_path2)
    plt.close(fig2)
    logging.info(f"📊 Saved proprioception and action plot to {save_path2}")

    # ===============================================
    # === PLOT 3: Visual Feature Correlation Heatmap ===
    # ===============================================
    sample_size = min(5000, X_train.shape[0])
    sample_indices = np.random.choice(X_train.shape[0], sample_size, replace=False)
    X_sample = X_train[sample_indices]
    correlation_matrix = np.corrcoef(X_sample, rowvar=False)

    fig3, ax = plt.subplots(figsize=(14, 12))
    sns.heatmap(correlation_matrix, cmap='viridis', ax=ax)
    ax.set_title('Feature Correlation Matrix (Sample from Training Set)')
    ax.set_xlabel('Feature Index'); ax.set_ylabel('Feature Index')
    ax.axvline(x=tactile_total_dim, color='white', linestyle='--', linewidth=2)
    ax.axhline(y=tactile_total_dim, color='white', linestyle='--', linewidth=2)
    ax.axvline(x=joint_start_idx, color='white', linestyle='--', linewidth=2)
    ax.axhline(y=joint_start_idx, color='white', linestyle='--', linewidth=2)
    ax.text(tactile_total_dim/2, -10, 'Tactile', ha='center', color='white', weight='bold')
    ax.text(tactile_total_dim + visual_dim/2, -10, 'Visual', ha='center', color='white', weight='bold')
    ax.text(joint_start_idx + proprio_dim/2, -10, 'Joints', ha='center', color='white', weight='bold')

    plt.tight_layout()
    save_path3 = output_dir / "feature_correlation_heatmap.png"
    plt.savefig(save_path3)
    plt.close(fig3)
    logging.info(f"📊 Saved feature correlation heatmap to {save_path3}")

def split_and_save_dataset(input_path, output_path, split_ratio=0.8, seed=42):
    """
    Loads a dataset of trajectories, splits them into training and validation sets,
    and saves the final concatenated arrays.
    
    Args:
        input_path (Path): Path to the input .pkl file containing a list of trajectories.
        output_path (Path): Path to save the output .npz file.
        split_ratio (float): The proportion of trajectories to use for training.
        seed (int): A random seed for reproducible shuffling.
    """
    if not input_path.exists():
        logging.error(f"❌ Input dataset file not found: {input_path}")
        return

    # --- 1. Load the list of trajectories from the .pkl file ---
    logging.info(f"Loading trajectories from {input_path}...")
    with open(input_path, "rb") as f:
        all_trajectories = pickle.load(f)

    num_trajectories = len(all_trajectories)
    if num_trajectories < 2:
        logging.error("❌ Need at least 2 trajectories to perform a split.")
        return
        
    logging.info(f"Loaded {num_trajectories} trajectories.")

    # --- 2. Shuffle the trajectories for a random split ---
    # This is a critical step to ensure the training and validation sets
    # are representative of the same data distribution.
    random.seed(seed)
    random.shuffle(all_trajectories)
    logging.info(f"Shuffled trajectories with random seed {seed}.")

    # --- 3. Split trajectories into training and validation sets ---
    split_index = int(num_trajectories * split_ratio)
    train_trajectories = all_trajectories[:split_index]
    val_trajectories = all_trajectories[split_index:]

    logging.info(f"Splitting into {len(train_trajectories)} training trajectories and {len(val_trajectories)} validation trajectories.")
    if not val_trajectories:
        logging.warning("Validation set is empty! Consider a smaller split_ratio or more data.")

    # --- 4. Concatenate data from the split trajectories ---
    def concatenate_trajectories(trajectories):
        """Helper function to combine a list of trajectory dictionaries into final numpy arrays."""
        if not trajectories:
            return None, None, None, None

        tactile_t = np.concatenate([traj['tactile_t'] for traj in trajectories], axis=0)
        visual_t = np.concatenate([traj['visual_t'] for traj in trajectories], axis=0)
        joints_t = np.concatenate([traj['joints_t'] for traj in trajectories], axis=0)
        delta_q = np.concatenate([traj['delta_q'] for traj in trajectories], axis=0)
        
        return tactile_t, visual_t, joints_t, delta_q

    tactile_train, visual_train, joints_train, delta_q_train = concatenate_trajectories(train_trajectories)
    tactile_val, visual_val, joints_val, delta_q_val = concatenate_trajectories(val_trajectories)

    # --- 5. Combine modalities into final X and y arrays ---
    X_train = np.concatenate([tactile_train, visual_train, joints_train], axis=1)
    y_train = delta_q_train

    logging.info(f"Train split final shapes: X_train={X_train.shape}, y_train={y_train.shape}")
    
    if val_trajectories:
        X_val = np.concatenate([tactile_val, visual_val, joints_val], axis=1)
        y_val = delta_q_val
        logging.info(f"Validation split final shapes: X_val={X_val.shape}, y_val={y_val.shape}")
    else:
        # Create empty arrays if validation set is empty to avoid errors
        X_val = np.empty((0, X_train.shape[1]), dtype=X_train.dtype)
        y_val = np.empty((0, y_train.shape[1]), dtype=y_train.dtype)

    # --- 6. Visualize the distributions ---
    if X_val.shape[0] > 0:
        # The output plot will be saved in the same directory as the .npz file
        visualize_split_distributions(X_train, y_train, X_val, y_val, output_path.parent)
    # --- 7. Save the final split dataset ---
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_path,
        X_train=X_train,
        y_train=y_train,
        X_val=X_val,
        y_val=y_val
    )
    logging.info(f"✅ Split dataset saved to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Split a trajectory dataset into training and validation sets.")
    parser.add_argument(
        "--input_file", 
        type=str, 
        required=True, 
        help="Path to the input .pkl dataset file from dataset_builder.py."
    )
    parser.add_argument(
        "--output_file", 
        type=str, 
        required=True, 
        help="Path to save the final split .npz dataset file."
    )
    parser.add_argument(
        "--split_ratio", 
        type=float, 
        default=0.8, 
        help="Fraction of trajectories to use for the training set (e.g., 0.8 for 80%%)."
    )
    parser.add_argument(
        "--seed", 
        type=int, 
        default=42, 
        help="Random seed for shuffling to ensure reproducibility."
    )
    args = parser.parse_args()

    split_and_save_dataset(
        input_path=Path(args.input_file),
        output_path=Path(args.output_file),
        split_ratio=args.split_ratio,
        seed=args.seed
    )

if __name__ == "__main__":
    # Example usage from your terminal:
    # python split_dataset.py --input_file data/processed/tube_insertion_dataset.pkl --output_file data/processed/dataset_split.npz
    main()