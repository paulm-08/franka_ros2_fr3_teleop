#!/usr/bin/env python3
import numpy as np
import torch
import matplotlib.pyplot as plt
import logging, os, argparse, pickle, random
from pathlib import Path
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

from model_pipeline.train import build_model, MLPPolicy, LSTMPolicy, GRUPolicy, TransformerPolicy  # Assuming train.py is in the same package
from model_pipeline import paths

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def to_np(tensor):
    return tensor.detach().cpu().numpy()

# In evaluate_policy.py

def perform_rollout(model, trajectory, horizon, norm_stats, joint_limits, frame_stack_k, device):
    """Performs a plausible closed-loop rollout on a single trajectory, compatible with all model types."""
    X_mean, X_std, y_mean, y_std = norm_stats
    joint_min, joint_max = joint_limits

    X_traj_unstacked = torch.tensor(
        np.concatenate([trajectory['tactile_t'], trajectory['visual_t'], trajectory['joints_t']], axis=1),
        dtype=torch.float32, device=device
    )
    
    joint_dim = trajectory['joints_t'].shape[1]
    single_frame_dim = X_traj_unstacked.shape[1]
    JOINT_START_IDX = single_frame_dim - joint_dim

    rollout_steps = min(horizon, len(X_traj_unstacked) - frame_stack_k)
    if rollout_steps <= 0: return None, None, None
    
    q_pred_history = X_traj_unstacked[:frame_stack_k, JOINT_START_IDX:].clone()
    predicted_q_trajectory = []

    # Check if the loaded model is an MLP to decide how to format data
    is_sequence_model = not isinstance(model, MLPPolicy)

    for i in range(rollout_steps):
        t = i + frame_stack_k - 1
        sensory_gt_stack = X_traj_unstacked[t - frame_stack_k + 1 : t + 1, :JOINT_START_IDX]
        state_window = torch.cat([sensory_gt_stack, q_pred_history], dim=1)
        
        # --- FIX: Prepare input batch correctly based on model type ---
        if is_sequence_model:
            # For LSTM/GRU, normalize each frame and keep the sequence shape
            state_norm = (state_window - X_mean) / X_std
            state_norm = state_norm.unsqueeze(0) # Add batch dimension -> (1, K, D)
        else: # For MLP
            # For MLP, flatten the sequence and then normalize
            state_flattened = state_window.flatten()
            X_mean_stacked = X_mean.repeat(frame_stack_k)
            X_std_stacked = X_std.repeat(frame_stack_k)
            state_norm = (state_flattened - X_mean_stacked) / X_std_stacked
            state_norm = state_norm.unsqueeze(0) # Add batch dimension -> (1, K*D)
        
        with torch.no_grad():
            pred_norm = model(state_norm)
            delta_q_pred = (pred_norm.view(-1) * y_std) + y_mean
            
        q_pred_next = q_pred_history[-1] + delta_q_pred
        q_pred_next = torch.clamp(q_pred_next, min=joint_min, max=joint_max)
        
        predicted_q_trajectory.append(to_np(q_pred_next))
        
        q_pred_history = torch.roll(q_pred_history, shifts=-1, dims=0)
        q_pred_history[-1] = q_pred_next

    pred_np = np.array(predicted_q_trajectory)
    gt_np = to_np(X_traj_unstacked[frame_stack_k : rollout_steps + frame_stack_k, JOINT_START_IDX:])

    metrics = {'mse': mean_squared_error(gt_np, pred_np), 'mae': mean_absolute_error(gt_np, pred_np), 'r2': r2_score(gt_np, pred_np)}
    return pred_np, gt_np, metrics

def main():
    parser = argparse.ArgumentParser(description="Evaluate a trained policy with frame stacking awareness.")
    # Use dynamic paths as defaults
    parser.add_argument("--model", type=str, default=str(paths.POLICY_MODELS_DIR / "policy_mlp_best.pt"), help="Path to the trained model checkpoint (.pt file).")
    parser.add_argument("--dataset_pkl", type=str, default=str(paths.PROCESSED_DATA_DIR / "dataset_cleaned.pkl"), help="Path to the original trajectory dataset (.pkl file).")
    parser.add_argument("--rollout", action="store_true", help="Enable closed-loop rollout mode.")
    parser.add_argument("--horizon", type=int, default=500, help="Max number of rollout steps per trajectory.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducing the train/val split.")
    parser.add_argument("--split_ratio", type=float, default=0.85, help="Train/val split ratio to reproduce.")
    parser.add_argument("--width", type=int, default=256, help="Width of hidden layers for MLP if specified in model.")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    
    # --- Load Model, Stats, and Data ---
    model_path = Path(args.model).resolve()
    dataset_path = Path(args.dataset_pkl).resolve()

    checkpoint = torch.load(model_path, map_location=device, weights_only=False)
    frame_stack_k = checkpoint.get("frame_stack", 1)
    model_type = checkpoint["model_type"]
    logging.info(f"Model was trained with K={frame_stack_k}, type='{model_type}'. Evaluating accordingly.")
    
    model = build_model(model_type, checkpoint["input_dim"], checkpoint["output_dim"], width=args.width).to(device)
    model.load_state_dict(checkpoint["state_dict"]); model.eval()
    
    norm_stats = (torch.tensor(checkpoint["X_mean"], device=device), torch.tensor(checkpoint["X_std"], device=device),
                  torch.tensor(checkpoint["y_mean"], device=device), torch.tensor(checkpoint["y_std"], device=device))
    joint_dim = checkpoint["output_dim"]
    joint_limits = (torch.full((joint_dim,), -2*np.pi, device=device), torch.full((joint_dim,), 2*np.pi, device=device))
    logging.info(f"Loaded model and normalization stats from: {args.model}")
    logging.info(f"Input Dim: {checkpoint['input_dim']}, Output Dim: {checkpoint['output_dim']}")
    logging.info(f"Joint Limits: Min {joint_limits[0][0].item()}, Max {joint_limits[1][0].item()}")
    # logging.info(f"Normalization - X_mean: {norm_stats[0].cpu().numpy()}, X_std: {norm_stats[1].cpu().numpy()}")
    # logging.info(f"Normalization - y_mean: {norm_stats[2].cpu().numpy()}, y_std: {norm_stats[3].cpu().numpy()}")

    with open(dataset_path, "rb") as f: all_trajectories = pickle.load(f)
    random.seed(args.seed); random.shuffle(all_trajectories)
    split_index = int(len(all_trajectories) * args.split_ratio)
    val_trajectories = all_trajectories[split_index:]
    
    output_dir = paths.MODELS_DIR / "debug"
    output_dir.mkdir(parents=True, exist_ok=True)
    logging.info(f"Loaded {len(val_trajectories)} validation trajectories from dataset.")
    logging.info(f"Evaluation outputs will be saved to: {output_dir}")
    
    if args.rollout:
        # --- ROLLOUT MODE ---
        all_metrics, all_preds, all_gts = [], [], []
        rollout_plot_dir = output_dir / "rollouts"; rollout_plot_dir.mkdir(exist_ok=True)
        logging.info(f"Starting per-trajectory closed-loop rollout evaluation...")

        for i, trajectory in enumerate(val_trajectories):
            # FIX: Pass the frame_stack_k argument to the helper function
            pred_np, gt_np, metrics = perform_rollout(model, trajectory, args.horizon, norm_stats, joint_limits, frame_stack_k, device)
            all_metrics.append(metrics); all_preds.append(pred_np); all_gts.append(gt_np)
            logging.info(f"  Trajectory {i+1}/{len(val_trajectories)} | R²={metrics['r2']:.4f}")

            # Plot individual arm trajectories
            fig_arm, ax_arm = plt.subplots(figsize=(12, 6))
            num_arm_joints = min(7, joint_dim)
            for j in range(num_arm_joints):
                ax_arm.plot(gt_np[:, j], color=f'C{j}', linestyle='--', label=f'GT Joint {j}')
                ax_arm.plot(pred_np[:, j], color=f'C{j}', label=f'Pred Joint {j}')
            ax_arm.set_title(f'Rollout vs. GT - Arm Joints (Trajectory {i+1})'); ax_arm.set_xlabel('Timestep'); ax_arm.set_ylabel('Joint Angle (rad)')
            ax_arm.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0)); ax_arm.grid(True, linestyle='--')
            plt.tight_layout(); plt.savefig(rollout_plot_dir / f"rollout_traj_{i+1}_arm.png"); plt.close(fig_arm)

            # Plot individual hand trajectories
            if joint_dim > num_arm_joints:
                fig_hand, ax_hand = plt.subplots(figsize=(12, 6))
                for j in range(num_arm_joints, joint_dim):
                    ax_hand.plot(gt_np[:, j], color=f'C{j-num_arm_joints}', linestyle='--', label=f'GT Joint {j}')
                    ax_hand.plot(pred_np[:, j], color=f'C{j-num_arm_joints}', label=f'Pred Joint {j}')
                ax_hand.set_title(f'Rollout vs. GT - Hand Joints (Trajectory {i+1})'); ax_hand.set_xlabel('Timestep'); ax_hand.set_ylabel('Joint Angle (rad)')
                ax_hand.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0)); ax_hand.grid(True, linestyle='--')
                plt.tight_layout(); plt.savefig(rollout_plot_dir / f"rollout_traj_{i+1}_hand.png"); plt.close(fig_hand)

        # --- ADDED: Aggregate Drift and Per-Joint Error Plots ---
        if all_preds:
            # 1. Plot drift over time for the first trajectory as a sample
            drift_per_step = np.sqrt(np.mean((all_preds[0] - all_gts[0]) ** 2, axis=1))
            plt.figure(figsize=(10, 5))
            plt.plot(drift_per_step)
            plt.xlabel("Timestep in Rollout"); plt.ylabel("Root Mean Square Error (RMSE)")
            plt.title("Prediction Drift Over Time (Sample Trajectory 1)"); plt.grid(True)
            plt.savefig(output_dir / "rollout_drift_over_time.png"); plt.close()

            # 2. Plot average per-joint error across ALL trajectories
            all_preds_np = np.concatenate(all_preds, axis=0)
            all_gts_np = np.concatenate(all_gts, axis=0)
            per_joint_error = np.sqrt(np.mean((all_preds_np - all_gts_np) ** 2, axis=0))
            plt.figure(figsize=(12, 6))
            plt.bar(np.arange(joint_dim), per_joint_error)
            plt.xlabel("Joint Index"); plt.ylabel("Overall RMSE")
            plt.title("Per-Joint Rollout Error (Averaged Across All Validation Rollouts)"); plt.grid(True)
            plt.savefig(output_dir / "rollout_per_joint_error.png"); plt.close()
            logging.info(f"Aggregate rollout plots saved in: {output_dir}")

        # Report average metrics
        avg_metrics = {key: np.mean([m[key] for m in all_metrics]) for key in all_metrics[0]}
        logging.info("\n" + "="*50 + "\n          📊 AVERAGE ROLLOUT METRICS 📊\n" + "="*50)
        logging.info(f"Average MSE: {avg_metrics['mse']:.6f}\nAverage MAE: {avg_metrics['mae']:.6f}\nAverage R² : {avg_metrics['r2']:.4f}")
        logging.info(f"Individual trajectory plots saved in: {rollout_plot_dir}")

    else:
        # --- ONE-STEP MODE (Frame-Stack Aware & Corrected) ---
        logging.info("Starting one-step evaluation with frame stacking...")

        is_sequence_model = model_type in ["lstm", "gru", "transformer"]
        flatten_data = not is_sequence_model
        
        X_val_list, y_val_list = [], []
        for traj in val_trajectories:
            X_unstacked = np.concatenate([traj['tactile_t'], traj['visual_t'], traj['joints_t']], axis=1)
            y_unstacked = traj['delta_q']
            if len(X_unstacked) < frame_stack_k: continue
            
            for i in range(frame_stack_k - 1, len(X_unstacked)):
                state_sequence = X_unstacked[i - frame_stack_k + 1 : i + 1]
                X_val_list.append(state_sequence.flatten() if flatten_data else state_sequence)
                y_val_list.append(y_unstacked[i])

        X_val_raw = torch.tensor(np.array(X_val_list), dtype=torch.float32, device=device)
        y_val_true = torch.tensor(np.array(y_val_list), dtype=torch.float32, device=device)
        
        X_mean, X_std, y_mean, y_std = norm_stats

        with torch.no_grad():
            if flatten_data: # MLP
                X_val_norm = (X_val_raw - X_mean.repeat(frame_stack_k)) / X_std.repeat(frame_stack_k)
            else: # Sequence models
                X_val_norm = (X_val_raw - X_mean) / X_std

            pred_norm = model(X_val_norm)
            delta_q_pred = (pred_norm * y_std) + y_mean
        
        delta_q_pred_np = to_np(delta_q_pred)
        delta_q_true_np = to_np(y_val_true)
        
        metrics = {'mse': mean_squared_error(delta_q_true_np, delta_q_pred_np), 
                   'mae': mean_absolute_error(delta_q_true_np, delta_q_pred_np), 
                   'r2': r2_score(delta_q_true_np, delta_q_pred_np)}

        logging.info("\n" + "="*50 + "\n          📊 ONE-STEP (delta_q) METRICS 📊\n" + "="*50)
        logging.info(f"MSE: {metrics['mse']:.6f}\nMAE: {metrics['mae']:.6f}\nR² : {metrics['r2']:.4f}")
        logging.info(f"Ground truth stats - Mean: {delta_q_true_np.mean(axis=0)}, Std: {delta_q_true_np.std(axis=0)}")
        logging.info(f"Predicted stats    - Mean: {delta_q_pred_np.mean(axis=0)}, Std: {delta_q_pred_np.std(axis=0)}")
        logging.info(f"Difference stats   - Mean: {(delta_q_pred_np - delta_q_true_np).mean(axis=0)}, Std: {(delta_q_pred_np - delta_q_true_np).std(axis=0)}")
        logging.info(f"Total one-step samples evaluated: {len(delta_q_true_np)}")

        # --- FIX: Update plotting to use the correct variables and labels ---
        
        # PLOT 1: Multi-Joint Scatter Plot
        joints_to_plot = {'Arm Base': 0, 'Arm Elbow': 4, 'Arm Wrist': 6, 'Hand Finger': 20}
        fig, axes = plt.subplots(2, 2, figsize=(12, 12))
        fig.suptitle('One-Step Action Prediction (Δq) vs. Ground Truth', fontsize=16)
        axes = axes.flatten()

        for i, (name, joint_idx) in enumerate(joints_to_plot.items()):
            if joint_idx >= joint_dim: continue
            ax = axes[i]
            # Use the correct delta_q variables for plotting
            ax.scatter(delta_q_true_np[:, joint_idx], delta_q_pred_np[:, joint_idx], alpha=0.1)
            mn, mx = delta_q_true_np[:, joint_idx].min(), delta_q_true_np[:, joint_idx].max()
            ax.plot([mn, mx], [mn, mx], "r--", label='Perfect Prediction')
            ax.set_title(f'Joint {joint_idx} ({name})')
            ax.set_xlabel('Ground Truth Action (Δq)'); ax.set_ylabel('Predicted Action (Δq)')
            ax.grid(True); ax.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig(output_dir / "onestep_eval_scatter_multi.png"); plt.close()

        # PLOT 2: Error Histogram
        errors = delta_q_pred_np - delta_q_true_np
        plt.figure(figsize=(10, 5)); plt.hist(errors.flatten(), bins=100)
        plt.xlabel("Action Prediction Error (Δq, in rad)"); plt.ylabel("Frequency"); plt.title("One-Step Prediction Error Distribution")
        plt.grid(True); plt.tight_layout()
        plt.savefig(output_dir / "onestep_eval_error_hist.png"); plt.close()
        
        logging.info(f"Evaluation plots saved in: {output_dir}")

if __name__ == "__main__":
    main()