#!/usr/bin/env python3
import numpy as np
import torch
import matplotlib.pyplot as plt
import logging
import os
import argparse
import pickle
import random
from pathlib import Path
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import seaborn as sns

# Assuming your model definitions and build_model function are in a shared file
from model_pipeline.train import build_model

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def to_np(tensor):
    return tensor.detach().cpu().numpy()

def perform_rollout(model, trajectory, horizon, norm_stats, joint_limits, device):
    """Performs a plausible closed-loop rollout on a single trajectory."""
    X_mean, X_std, y_mean, y_std = norm_stats
    joint_min, joint_max = joint_limits
    
    # FIX: Pass device explicitly instead of using model.device
    X_traj = torch.tensor(
        np.concatenate([trajectory['tactile_t'], trajectory['visual_t'], trajectory['joints_t']], axis=1),
        dtype=torch.float32, device=device
    )
    joint_dim = trajectory['joints_t'].shape[1]
    JOINT_START_IDX = X_traj.shape[1] - joint_dim

    rollout_steps = min(horizon, len(X_traj) - 1)
    q_pred_current = X_traj[0, JOINT_START_IDX:].clone()
    
    predicted_q_trajectory = []
    for t in range(rollout_steps):
        sensory_gt_current = X_traj[t, :JOINT_START_IDX]
        x_in = torch.cat((sensory_gt_current, q_pred_current), dim=0)
        x_in_norm = (x_in - X_mean.squeeze(0)) / X_std.squeeze(0)
        
        with torch.no_grad():
            delta_q_pred_norm = model(x_in_norm.unsqueeze(0))
            delta_q_pred = (delta_q_pred_norm.squeeze(0) * y_std) + y_mean
            
        q_pred_next = q_pred_current + delta_q_pred
        q_pred_next = torch.clamp(q_pred_next, min=joint_min, max=joint_max)
        
        predicted_q_trajectory.append(to_np(q_pred_next))
        q_pred_current = q_pred_next

    pred_np = np.array(predicted_q_trajectory)
    gt_np = to_np(X_traj[1 : rollout_steps + 1, JOINT_START_IDX:])

    metrics = {'mse': mean_squared_error(gt_np, pred_np), 'mae': mean_absolute_error(gt_np, pred_np), 'r2': r2_score(gt_np, pred_np)}
    return pred_np, gt_np, metrics

def main():
    parser = argparse.ArgumentParser(description="Evaluate a trained policy in one-step or rollout mode.")
    parser.add_argument("--model", type=str, required=True, help="Path to the trained model checkpoint (.pt file).")
    parser.add_argument("--dataset_pkl", type=str, required=True, help="Path to the original trajectory dataset (.pkl file).")
    parser.add_argument("--rollout", action="store_true", help="Enable closed-loop rollout mode.")
    parser.add_argument("--horizon", type=int, default=500, help="Max number of rollout steps per trajectory.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducing the train/val split.")
    parser.add_argument("--split_ratio", type=float, default=0.85, help="Train/val split ratio to reproduce.")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    
    # --- Load Model, Stats, and Data ---
    checkpoint = torch.load(args.model, map_location=device, weights_only=False)
    model = build_model(checkpoint["model_type"], checkpoint["input_dim"], checkpoint["output_dim"]).to(device)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    norm_stats = (torch.tensor(checkpoint["X_mean"], device=device), torch.tensor(checkpoint["X_std"], device=device),
                  torch.tensor(checkpoint["y_mean"], device=device), torch.tensor(checkpoint["y_std"], device=device))
    joint_dim = checkpoint["output_dim"]
    joint_limits = (torch.full((joint_dim,), -2*np.pi, device=device), torch.full((joint_dim,), 2*np.pi, device=device))

    with open(args.dataset_pkl, "rb") as f: all_trajectories = pickle.load(f)
    random.seed(args.seed); random.shuffle(all_trajectories)
    split_index = int(len(all_trajectories) * args.split_ratio)
    val_trajectories = all_trajectories[split_index:]
    
    output_dir = Path("data/debug/")
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.rollout:
        # --- ROLLOUT MODE ---
        all_metrics, all_preds, all_gts = [], [], []
        rollout_plot_dir = output_dir / "rollouts"; rollout_plot_dir.mkdir(exist_ok=True)
        logging.info(f"Starting per-trajectory closed-loop rollout evaluation...")

        for i, trajectory in enumerate(val_trajectories):
            pred_np, gt_np, metrics = perform_rollout(model, trajectory, args.horizon, norm_stats, joint_limits, device=device)
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
        # --- ONE-STEP MODE ---
        # (Data concatenation and prediction logic is the same)
        X_val_list = [np.concatenate([t['tactile_t'], t['visual_t'], t['joints_t']], axis=1) for t in val_trajectories]
        y_val_list = [t['delta_q'] for t in val_trajectories]
        X_val = torch.tensor(np.concatenate(X_val_list, axis=0), dtype=torch.float32, device=device)
        y_val = torch.tensor(np.concatenate(y_val_list, axis=0), dtype=torch.float32, device=device)
        
        with torch.no_grad():
            X_val_norm = (X_val - norm_stats[0]) / norm_stats[1]
            delta_q_pred_norm = model(X_val_norm)
            delta_q_pred = delta_q_pred_norm * norm_stats[3] + norm_stats[2]
        
        JOINT_START_IDX = X_val.shape[1] - joint_dim
        q_current = X_val[:, JOINT_START_IDX:]
        q_true = q_current + y_val
        q_pred = q_current + delta_q_pred
        q_pred_np, q_true_np = to_np(q_pred), to_np(q_true)
        metrics = {'mse': mean_squared_error(q_true_np, q_pred_np), 'mae': mean_absolute_error(q_true_np, q_pred_np), 'r2': r2_score(q_true_np, q_pred_np)}
        
        logging.info("\n" + "="*50 + "\n          📊 ONE-STEP EVALUATION METRICS 📊\n" + "="*50)
        logging.info(f"MSE: {metrics['mse']:.6f}\nMAE: {metrics['mae']:.6f}\nR² : {metrics['r2']:.4f}")
        
        # --- ADDED: Enhanced visualization plots for one-step evaluation ---
        
        # PLOT 1: Multi-Joint Scatter Plot
        joints_to_plot = {'Arm Base': 0, 'Arm Elbow': 4, 'Arm Wrist': 6, 'Hand Finger': 20}
        fig, axes = plt.subplots(2, 2, figsize=(12, 12))
        fig.suptitle('One-Step Prediction vs. Ground Truth for Different Joints', fontsize=16)
        axes = axes.flatten()

        for i, (name, joint_idx) in enumerate(joints_to_plot.items()):
            if joint_idx >= joint_dim: continue
            ax = axes[i]
            ax.scatter(q_true_np[:, joint_idx], q_pred_np[:, joint_idx], alpha=0.1)
            mn, mx = q_true_np[:, joint_idx].min(), q_true_np[:, joint_idx].max()
            ax.plot([mn, mx], [mn, mx], "r--", label='Perfect Prediction')
            ax.set_title(f'Joint {joint_idx} ({name})'); ax.set_xlabel('Ground Truth'); ax.set_ylabel('Predicted')
            ax.grid(True); ax.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig(output_dir / "onestep_eval_scatter_multi.png"); plt.close()

        # PLOT 2: Error Histogram (remains the same)
        errors = q_pred_np - q_true_np
        plt.figure(figsize=(10, 5)); plt.hist(errors.flatten(), bins=100)
        plt.xlabel("Prediction Error (rad)"); plt.ylabel("Frequency"); plt.title("One-Step Prediction Error Distribution")
        plt.grid(True); plt.tight_layout()
        plt.savefig(output_dir / "onestep_eval_error_hist.png"); plt.close()
        
        logging.info(f"Evaluation plots saved in: {output_dir}")
if __name__ == "__main__":
    main()