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

def plot_visual_distributions(X_train, X_val, tactile_total_dim, visual_dim, output_dir, config):
    logging.info("Parsing visual feature vector based on config...")
    state_config = config.get('state', {})
    control_mode = config.get('control_mode', 'joint_space')

    use_kps = state_config.get('use_keypoint_extractor', False)
    use_3d = state_config.get('use_3d_keypoints', False)
    use_embs = state_config.get('use_resnet_embeddings', False)
    use_3d_tactile = state_config.get('use_3d_tactile', False)

    if use_kps:
        kp_dim_per_obj = (3 if use_3d else 2) + 2  # (pos dims) + conf + flag
        rel_vec_dim = 3 if use_3d else 2
        kp_dim_per_cam = (kp_dim_per_obj * 2) + rel_vec_dim
    else:
        # ensure these variables exist to avoid NameError later
        kp_dim_per_obj = 0
        rel_vec_dim = 0
        kp_dim_per_cam = 0

    if use_embs:
        rgb_dim = config.get('visual_dim', 32)
        depth_dim = config.get('depth_dim', 16)
    else:
        rgb_dim = depth_dim = 0

    single_cam_dim = (kp_dim_per_cam if use_kps else 0) + (rgb_dim + depth_dim if use_embs else 0)
    expected_visual_dim = single_cam_dim * 2

    if visual_dim != expected_visual_dim:
        logging.error(f"Visual dimension mismatch! Data has {visual_dim}D, but config implies {expected_visual_dim}D.")
        return

    # --- unified base index for camera features ---
    tactile_3d_dim = 21 if use_3d_tactile else 0
    proprio_dim = 23 + tactile_3d_dim  # keep this explicit and documented
    base_idx = tactile_total_dim + proprio_dim
    c1_start = base_idx
    c2_start = base_idx + single_cam_dim

    # --- keypoint indices (if enabled) ---
    if use_kps:
        # c1 indices
        if use_3d:
            c1_tube_x, c1_tube_y, c1_tube_z = c1_start, c1_start+1, c1_start+2
            c1_tube_conf, c1_tube_flag = c1_start+3, c1_start+4
            peg_offset = kp_dim_per_obj
            c1_peg_x, c1_peg_y, c1_peg_z = c1_start + peg_offset, c1_start + peg_offset + 1, c1_start + peg_offset + 2
            c1_peg_conf, c1_peg_flag = c1_start + peg_offset + 3, c1_start + peg_offset + 4
        else:
            c1_tube_x, c1_tube_y = c1_start, c1_start+1
            c1_tube_conf, c1_tube_flag = c1_start+2, c1_start+3
            peg_offset = kp_dim_per_obj
            c1_peg_x, c1_peg_y = c1_start + peg_offset, c1_start + peg_offset + 1
            c1_peg_conf, c1_peg_flag = c1_start + peg_offset + 2, c1_start + peg_offset + 3

        c1_rel_start = c1_start + (kp_dim_per_obj * 2)

        # c2 indices mirror c1 but offset by single_cam_dim
        if use_3d:
            c2_tube_x, c2_tube_y, c2_tube_z = c2_start, c2_start+1, c2_start+2
            c2_tube_conf, c2_tube_flag = c2_start+3, c2_start+4
            c2_peg_x, c2_peg_y, c2_peg_z = c2_start + peg_offset, c2_start + peg_offset + 1, c2_start + peg_offset + 2
            c2_peg_conf, c2_peg_flag = c2_start + peg_offset + 3, c2_start + peg_offset + 4
        else:
            c2_tube_x, c2_tube_y = c2_start, c2_start+1
            c2_tube_conf, c2_tube_flag = c2_start+2, c2_start+3
            c2_peg_x, c2_peg_y = c2_start + peg_offset, c2_start + peg_offset + 1
            c2_peg_conf, c2_peg_flag = c2_start + peg_offset + 2, c2_start + peg_offset + 3

        c2_rel_start = c2_start + (kp_dim_per_obj * 2)

    # --- embedding starts (now consistent with base_idx) ---
    if use_embs:
        c1_rgb_emb_start = base_idx + (kp_dim_per_cam if use_kps else 0)
        c1_depth_emb_start = c1_rgb_emb_start + rgb_dim
        c2_rgb_emb_start = c2_start + (kp_dim_per_cam if use_kps else 0)
        c2_depth_emb_start = c2_rgb_emb_start + rgb_dim

    # --- plotting layout (unchanged) ---
    num_rows = 0
    if use_kps: num_rows += 8
    if use_embs: num_rows += 4
    if num_rows == 0:
        logging.warning("No visual features enabled in config."); return

    fig = plt.figure(figsize=(20, 5 * num_rows))
    gs = fig.add_gridspec(num_rows, 2)
    fig.suptitle('Hybrid Visual Feature Analysis (Config-Aware)', fontsize=18, y=1.01)
    current_row = 0

    if use_kps:
        # Row 1: detection rates
        ax_rate1 = fig.add_subplot(gs[current_row, 0])
        tube1_rate = float(np.mean(X_train[:, c1_tube_flag]) * 100) if X_train.shape[0] > 0 else 0.0
        peg1_rate = float(np.mean(X_train[:, c1_peg_flag]) * 100) if X_train.shape[0] > 0 else 0.0
        ax_rate1.bar(['Tube Tip', 'Peg'], [tube1_rate, peg1_rate])
        ax_rate1.set_title('Camera 1: Object Detection Rate (%)'); ax_rate1.set_ylim(0, 105)
        for i, rate in enumerate([tube1_rate, peg1_rate]): ax_rate1.text(i, rate + 2, f'{rate:.1f}%', ha='center')

        ax_rate2 = fig.add_subplot(gs[current_row, 1])
        tube2_rate = float(np.mean(X_train[:, c2_tube_flag]) * 100) if X_train.shape[0] > 0 else 0.0
        peg2_rate = float(np.mean(X_train[:, c2_peg_flag]) * 100) if X_train.shape[0] > 0 else 0.0
        ax_rate2.bar(['Tube Tip', 'Peg'], [tube2_rate, peg2_rate])
        ax_rate2.set_title('Camera 2: Object Detection Rate (%)'); ax_rate2.set_ylim(0, 105)
        for i, rate in enumerate([tube2_rate, peg2_rate]): ax_rate2.text(i, rate + 2, f'{rate:.1f}%', ha='center')
        current_row += 1

        # Row 2: confidence histograms (only where detected)
        def safe_hist(ax, arr, label):
            if arr.size == 0:
                ax.text(0.5, 0.5, f'No {label} detections', ha='center', va='center', transform=ax.transAxes)
            else:
                ax.hist(arr, bins=50, density=True, alpha=0.7, label=label)

        ax_conf1 = fig.add_subplot(gs[current_row, 0])
        safe_hist(ax_conf1, X_train[X_train[:, c1_tube_flag] > 0, c1_tube_conf], 'Tube Confidence (C1)')
        safe_hist(ax_conf1, X_train[X_train[:, c1_peg_flag] > 0, c1_peg_conf], 'Peg Confidence (C1)')
        ax_conf1.set_title('Camera 1: Detection Confidence (where detected)')
        ax_conf1.legend(); ax_conf1.grid(True, linestyle='--')

        ax_conf2 = fig.add_subplot(gs[current_row, 1])
        safe_hist(ax_conf2, X_train[X_train[:, c2_tube_flag] > 0, c2_tube_conf], 'Tube Confidence (C2)')
        safe_hist(ax_conf2, X_train[X_train[:, c2_peg_flag] > 0, c2_peg_conf], 'Peg Confidence (C2)')
        ax_conf2.set_title('Camera 2: Detection Confidence (where detected)'); ax_conf2.legend(); ax_conf2.grid(True, linestyle='--')
        current_row += 1

        # Row 3: position coverage (use only X,Y coords — project if 3D)
        ax_pos1 = fig.add_subplot(gs[current_row, 0])
        ax_pos2 = fig.add_subplot(gs[current_row, 1])

        # --- Camera 1 ---
        mask_c1_tube = X_train[:, c1_tube_flag] > 0
        mask_c1_peg  = X_train[:, c1_peg_flag]  > 0

        tx_idx, ty_idx = c1_tube_x, c1_tube_y
        px_idx, py_idx = c1_peg_x,  c1_peg_y

        if np.any(mask_c1_tube):
            ax_pos1.scatter(X_train[mask_c1_tube, tx_idx], X_train[mask_c1_tube, ty_idx],
                            alpha=0.1, label='Tube Tip', color='tab:blue')
        if np.any(mask_c1_peg):
            ax_pos1.scatter(X_train[mask_c1_peg, px_idx], X_train[mask_c1_peg, py_idx],
                            alpha=0.1, label='Peg', color='tab:orange')

        ax_pos1.set_title('Camera 1: Position Coverage (Detected Only)')
        ax_pos1.legend()
        ax_pos1.grid(True, linestyle='--')

        # Compute range for Camera 1
        if np.any(mask_c1_tube):
            c1_xy_min = X_train[mask_c1_tube][:, [c1_tube_x, c1_tube_y]].min(axis=0)
            c1_xy_max = X_train[mask_c1_tube][:, [c1_tube_x, c1_tube_y]].max(axis=0)
            ax_pos1.set_xlim(c1_xy_min[0], c1_xy_max[0])
            ax_pos1.set_ylim(c1_xy_min[1], c1_xy_max[1])

        print("Camera 1 tube xy range:", c1_xy_min, c1_xy_max)


        # --- Camera 2 ---
        mask_c2_tube = X_train[:, c2_tube_flag] > 0
        mask_c2_peg  = X_train[:, c2_peg_flag]  > 0

        tx_idx2, ty_idx2 = c2_tube_x, c2_tube_y
        px_idx2, py_idx2 = c2_peg_x,  c2_peg_y

        if np.any(mask_c2_tube):
            ax_pos2.scatter(X_train[mask_c2_tube, tx_idx2], X_train[mask_c2_tube, ty_idx2],
                            alpha=0.1, label='Tube Tip', color='tab:blue')
        if np.any(mask_c2_peg):
            ax_pos2.scatter(X_train[mask_c2_peg, px_idx2], X_train[mask_c2_peg, py_idx2],
                            alpha=0.1, label='Peg', color='tab:orange')

        ax_pos2.set_title('Camera 2: Position Coverage (Detected Only)')
        ax_pos2.legend()
        ax_pos2.grid(True, linestyle='--')

        # Compute range for Camera 2
        if np.any(mask_c2_tube):
            c2_xy_min = X_train[mask_c2_tube][:, [c2_tube_x, c2_tube_y]].min(axis=0)
            c2_xy_max = X_train[mask_c2_tube][:, [c2_tube_x, c2_tube_y]].max(axis=0)
            ax_pos2.set_xlim(c2_xy_min[0], c2_xy_max[0])
            ax_pos2.set_ylim(c2_xy_min[1], c2_xy_max[1])

        print("Camera 2 tube xy range:", c2_xy_min, c2_xy_max)
        current_row += 1

        # Row 4: relative position scatter (first two rel dims)
        ax_rel1 = fig.add_subplot(gs[current_row, 0])
        mask_c1_both = (X_train[:, c1_tube_flag] > 0) & (X_train[:, c1_peg_flag] > 0)
        if np.sum(mask_c1_both) > 0:
            rel_x1 = X_train[mask_c1_both, c1_rel_start]
            rel_y1 = X_train[mask_c1_both, c1_rel_start + 1]  # safe because rel_vec_dim >=2
            ax_rel1.scatter(rel_x1, rel_y1, alpha=0.1)
        else:
            ax_rel1.text(0.5, 0.5, 'No frames with both detections', ha='center', va='center', transform=ax_rel1.transAxes)
        ax_rel1.set_title(f'Camera 1: Relative Position ({np.sum(mask_c1_both)} frames)'); ax_rel1.grid(True, linestyle='--'); ax_rel1.axis('equal')
        current_row += 1

        ax_rel2 = fig.add_subplot(gs[current_row-1, 1])  # note: keep row pairing consistent
        mask_c2_both = (X_train[:, c2_tube_flag] > 0) & (X_train[:, c2_peg_flag] > 0)
        if np.sum(mask_c2_both) > 0:
            rel_x2 = X_train[mask_c2_both, c2_rel_start]
            rel_y2 = X_train[mask_c2_both, c2_rel_start + 1]
            ax_rel2.scatter(rel_x2, rel_y2, alpha=0.1)
        else:
            ax_rel2.text(0.5, 0.5, 'No frames with both detections', ha='center', va='center', transform=ax_rel2.transAxes)
        ax_rel2.set_title(f'Camera 2: Relative Position ({np.sum(mask_c2_both)} frames)'); ax_rel2.grid(True, linestyle='--'); ax_rel2.axis('equal')
        current_row += 1

    # --- Embeddings (if enabled) ---
    if use_embs:
        # choose safe embedding indices (avoid IndexError)
        def safe_kde(ax, arr_train, arr_val, label):
            if arr_train.size == 0 and arr_val.size == 0:
                ax.text(0.5, 0.5, f'No data for {label}', ha='center', va='center', transform=ax.transAxes)
                return
            if arr_train.size > 1:
                sns.kdeplot(arr_train, ax=ax, label='Train', fill=True)
            else:
                ax.hist(arr_train, bins=10, alpha=0.6, label='Train (count small)')
            if arr_val.size > 1:
                sns.kdeplot(arr_val, ax=ax, label='Val', fill=True, linestyle='--')
            elif arr_val.size == 1:
                ax.axvline(arr_val[0], linestyle='--', label='Val (single)')
            ax.legend()

        # pick sample embedding indices but guard range
        emb_idx_c1_rgb = c1_rgb_emb_start + min(5, max(0, rgb_dim-1))
        emb_idx_c2_rgb = c2_rgb_emb_start + min(10, max(0, rgb_dim-1))
        emb_idx_c1_depth = c1_depth_emb_start + min(5, max(0, depth_dim-1))
        emb_idx_c2_depth = c2_depth_emb_start + min(10, max(0, depth_dim-1))

        ax_emb_rgb1 = fig.add_subplot(gs[current_row, 0])
        safe_kde(ax_emb_rgb1, X_train[:, emb_idx_c1_rgb], X_val[:, emb_idx_c1_rgb], 'C1 RGB Emb')
        ax_emb_rgb1.set_title('Camera 1: Sample RGB Embedding Dists'); current_row += 0  # keep row pairing

        ax_emb_rgb2 = fig.add_subplot(gs[current_row, 1])
        safe_kde(ax_emb_rgb2, X_train[:, emb_idx_c2_rgb], X_val[:, emb_idx_c2_rgb], 'C2 RGB Emb')
        ax_emb_rgb2.set_title('Camera 2: Sample RGB Embedding Dists')
        current_row += 1

        ax_emb_d1 = fig.add_subplot(gs[current_row, 0])
        safe_kde(ax_emb_d1, X_train[:, emb_idx_c1_depth], X_val[:, emb_idx_c1_depth], 'C1 Depth Emb')
        ax_emb_d1.set_title('Camera 1: Sample Depth Embedding Dists')

        ax_emb_d2 = fig.add_subplot(gs[current_row, 1])
        safe_kde(ax_emb_d2, X_train[:, emb_idx_c2_depth], X_val[:, emb_idx_c2_depth], 'C2 Depth Emb')
        ax_emb_d2.set_title('Camera 2: Sample Depth Embedding Dists')
        current_row += 1

    # --- styling pass (use substring checks) ---
    for ax in fig.get_axes():
        t = ax.get_title()
        if 'Position Coverage' in t:
            ax.set_xlabel('X'); ax.set_ylabel('Y')
            ax.grid(True, linestyle='--'); ax.axis('equal')
            # ax.set_xlim(0, 1); ax.set_ylim(0, 1)
        elif 'Relative Position' in t:
            ax.set_xlabel('Relative X'); ax.set_ylabel('Relative Y')
            ax.grid(True, linestyle='--'); ax.axis('equal')
        elif 'Embedding' in t or 'Embedding Dists' in t:
            ax.grid(True, linestyle='--'); ax.set_xlabel('Feature Value'); ax.set_ylabel('Density')

    plt.tight_layout(rect=[0, 0, 1, 0.98])
    save_path = output_dir / "visual_feature_analysis.png"
    plt.savefig(save_path)
    plt.close(fig)
    logging.info(f"📊 Saved detailed visual plot to {save_path}")

def plot_proprio_action_distributions(X_train, y_train, X_val, y_val, joint_start_idx, output_dir, config):
    """
    Generates plots for proprioceptive state and action distributions.
    Now config-aware for task-space vs joint-space.
    """
    control_mode = config.get('control_mode', 'joint_space')
    
    # Define state features to plot
    state_indices_to_plot = {}
    if control_mode == 'task_space':
        state_indices_to_plot = {
            'arm_X': joint_start_idx, 'arm_Y': joint_start_idx + 1, 'arm_Z': joint_start_idx + 2
        }
    else: # joint_space
        state_indices_to_plot = {
            'arm_joint_0': joint_start_idx, 'arm_joint_2': joint_start_idx + 2, 'arm_joint_4': joint_start_idx + 4
        }
    state_indices_to_plot['hand_finger_1'] = joint_start_idx + (7 if control_mode == 'task_space' else 7) + 10 # Example hand joint
    
    # Define action features to plot
    action_indices_to_plot = {}
    if control_mode == 'task_space':
        action_indices_to_plot = {
            'action_dx': 0, 'action_dy': 1, 'action_dz': 2, 'action_hand_1': 6+10
        }
    else: # joint_space
        action_indices_to_plot = {
            'action_joint_0': 0, 'action_joint_2': 2, 'action_joint_4': 4, 'action_hand_1': 7+10
        }

    fig, axes = plt.subplots(2, len(state_indices_to_plot), figsize=(20, 10))
    fig.suptitle('Proprioceptive State and Action Distributions', fontsize=18, y=1.02)

    for i, (name, idx) in enumerate(state_indices_to_plot.items()):
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

def visualize_and_split_dataset(input_path, output_path, split_ratio=0.8, seed=42, config=None):
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
    # The .pkl file now contains 'state_t', 'goal_t', 'action_t'
    X_train = np.concatenate([t['state_t'] for t in train_trajectories], axis=0)
    y_train = np.concatenate([t['action_t'] for t in train_trajectories], axis=0)
    X_val = np.concatenate([t['state_t'] for t in val_trajectories], axis=0)
    y_val = np.concatenate([t['action_t'] for t in val_trajectories], axis=0)

    # --- 3. Dynamic Dimension Calculation for Plotting ---
    # We must parse the state vector based on the config
    state_config = config.get('state', {})
    control_mode = config.get('control_mode', 'joint_space')

    current_idx = 0
    tactile_total_dim = len(SENSOR_ORDER) * TACTILE_FEATURE_DIM
    current_idx += tactile_total_dim
    
    arm_proprio_dim = 7 # 7D for pose or 7D for joints
    joint_start_idx = current_idx # This is the start of all proprio
    current_idx += arm_proprio_dim
    
    hand_proprio_dim = 16
    current_idx += hand_proprio_dim
    
    if state_config.get('use_3d_tactile'):
        current_idx += len(config['kinematics']['tactile_frames']) * 7

    visual_dim = X_train.shape[1] - current_idx
    logging.info(f"Data dimensions: Tactile={tactile_total_dim}, Visual={visual_dim}, Proprio(Arm+Hand+Tactile)={X_train.shape[1] - tactile_total_dim - visual_dim}")

    # --- 4. Generate All Visualizations ---
    plot_dir = Path(output_path).parent if output_path else Path("./data/visualizations")
    plot_dir.mkdir(parents=True, exist_ok=True)
    logging.info(f"Generating visualizations in {plot_dir}...")
    
    if X_val.shape[0] > 0:
        plot_tactile_distributions(X_train, X_val, plot_dir)
        plot_visual_distributions(X_train, X_val, tactile_total_dim, visual_dim, plot_dir, config)
        plot_proprio_action_distributions(X_train, y_train, X_val, y_val, joint_start_idx, plot_dir, config)
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
        
        config_choices = find_config_files(paths.CONFIG_DIR)
        if not config_choices:
            logging.error(f"No .yaml config files found in {paths.CONFIG_DIR}. Exiting."); return

        questions = [
            inquirer.List('input_file',
                          message="Select the processed dataset (.pkl) to analyze and split",
                          choices=pkl_choices),
            inquirer.List('config_file',
                          message="Select the configuration file used to build this dataset",
                          choices=config_choices),
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

        config_path_rel = answers['config_file']
        config_path_abs = paths.WORKSPACE_ROOT / config_path_rel
        with open(config_path_abs, 'r') as f:
            config = yaml.safe_load(f)
        logging.info(f"Loaded configuration from {config_path_abs}")


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
        seed=42, # Using a fixed seed for consistency
        config=config
    )

if __name__ == "__main__":
    main()