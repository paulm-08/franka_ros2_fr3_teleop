#!/usr/bin/env python3
import numpy as np
import torch
import matplotlib.pyplot as plt
import logging, os, argparse, pickle, random
from pathlib import Path
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import inquirer

from model_pipeline.train import build_model, MLPPolicy, LSTMPolicy, GRUPolicy, TransformerPolicy  # Assuming train.py is in the same package
from model_pipeline import paths

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def to_np(tensor):
    return tensor.detach().cpu().numpy()

def find_policy_models(search_path):
    """Finds all policy checkpoint files in the specified directory."""
    logging.info(f"Searching for trained policy models (.pt) in: {search_path}...")
    # Find all files ending in _best.pt
    found_files = [p.relative_to(paths.WORKSPACE_ROOT) for p in search_path.glob("*_best.pt")]
    logging.info(f"Found {len(found_files)} models.")
    return [str(p) for p in sorted(found_files)]

def find_pkl_files(search_path):
    """Finds all .pkl dataset files, prioritizing cleaned files."""
    logging.info(f"Searching for processed datasets (.pkl) in: {search_path}...")
    all_files = [p.relative_to(paths.WORKSPACE_ROOT) for p in search_path.glob("*.pkl")]
    cleaned_files = sorted([p for p in all_files if 'cleaned' in str(p)])
    other_files = sorted([p for p in all_files if 'cleaned' not in str(p)])
    return [str(p) for p in cleaned_files + other_files]

def perform_rollout(model, trajectory, horizon, norm_stats, joint_limits, frame_stack_k, is_arm_only, num_arm_joints, device):
    """Performs a plausible closed-loop rollout, compatible with all model types and arm_only mode."""
    X_mean, X_std, y_mean, y_std = norm_stats
    joint_min, joint_max = joint_limits

    # The full, un-sliced trajectory data
    current_state_unstacked = torch.tensor(np.concatenate([trajectory['tactile_t'], trajectory['visual_t'], trajectory['joints_t']], axis=1), dtype=torch.float32, device=device)
    goal_state_unstacked = torch.tensor(trajectory['goal_t'][0], dtype=torch.float32, device=device)
    
    joint_dim_full = trajectory['joints_t'].shape[1]
    single_frame_dim_full = current_state_unstacked.shape[1]
    JOINT_START_IDX_FULL = single_frame_dim_full - joint_dim_full

    rollout_steps = min(horizon, len(current_state_unstacked) - frame_stack_k)
    if rollout_steps <= 0: return None, None, None
    
    q_pred_history = current_state_unstacked[:frame_stack_k, JOINT_START_IDX_FULL:].clone()
    predicted_q_trajectory = []

    is_sequence_model = not isinstance(model, MLPPolicy)

    for i in range(rollout_steps):
        t = i + frame_stack_k - 1
        sensory_gt_stack = current_state_unstacked[t - frame_stack_k + 1 : t + 1, :JOINT_START_IDX_FULL]
        
        # --- PREPARE STATE (This logic must perfectly match train.py) ---
        state_window = torch.cat([sensory_gt_stack, q_pred_history], dim=1)
        goal_window = goal_state_unstacked.unsqueeze(0).repeat(frame_stack_k, 1)
        full_state_sequence = torch.cat([state_window, goal_window], dim=1)
        
        # Apply normalization
        if is_sequence_model:
            state_norm = (full_state_sequence - X_mean) / X_std
            state_norm = state_norm.unsqueeze(0)
        else: # MLP
            state_norm = (full_state_sequence.flatten() - X_mean.repeat(frame_stack_k)) / X_std.repeat(frame_stack_k)
            state_norm = state_norm.unsqueeze(0)
        
        with torch.no_grad():
            action_norm = model(state_norm).squeeze(0)
            delta_q_pred_arm = (action_norm * y_std) + y_mean # This will be 7-DOF if arm_only
            
        # Reconstruct full action if in arm_only mode
        if is_arm_only:
            hand_zeros = torch.zeros(joint_dim_full - num_arm_joints, device=device)
            full_delta_q = torch.cat([delta_q_pred_arm, hand_zeros])
        else:
            full_delta_q = delta_q_pred_arm
            
        q_pred_next = q_pred_history[-1] + full_delta_q
        q_pred_next = torch.clamp(q_pred_next, min=joint_min, max=joint_max)
        
        predicted_q_trajectory.append(to_np(q_pred_next))
        q_pred_history = torch.roll(q_pred_history, shifts=-1, dims=0); q_pred_history[-1] = q_pred_next

    pred_np = np.array(predicted_q_trajectory)
    gt_np = to_np(current_state_unstacked[frame_stack_k : rollout_steps + frame_stack_k, JOINT_START_IDX_FULL:])

    if is_arm_only:
        # If in arm-only mode, slice both trajectories down to just the arm joints for scoring
        gt_for_metrics = gt_np[:, :num_arm_joints]
        pred_for_metrics = pred_np[:, :num_arm_joints]
        logging.info(f"  (Calculating metrics on the first {num_arm_joints} arm joints only)")
    else:
        # If controlling the whole body, use all joints for scoring
        gt_for_metrics = gt_np
        pred_for_metrics = pred_np

    metrics = {
        'mse': mean_squared_error(gt_for_metrics, pred_for_metrics),
        'mae': mean_absolute_error(gt_for_metrics, pred_for_metrics),
        'r2': r2_score(gt_for_metrics, pred_for_metrics)
    }
    
    # We still return the full 23-DOF trajectories for comprehensive plotting
    return pred_np, gt_np, metrics

def main():
    parser = argparse.ArgumentParser(description="Interactively evaluate a trained policy.")
    # Arguments are now fully optional, for advanced/scripted use
    parser.add_argument("--model", type=str, help="Optional: Directly provide a path to the model file.")
    parser.add_argument("--dataset_pkl", type=str, help="Optional: Directly provide a path to the dataset file.")
    parser.add_argument("--rollout", action="store_true", help="Optional: Force rollout mode.")
    parser.add_argument("--horizon", type=int, default=500)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--split_ratio", type=float, default=0.85)
    args = parser.parse_args()

    try:
        # --- 1. Interactively Select Model and Dataset ---
        answers = {}
        if not args.model:
            model_choices = find_policy_models(paths.POLICY_MODELS_DIR)
            if not model_choices: 
                logging.error(f"No models found in {paths.POLICY_MODELS_DIR}. Exiting.")
                return
            questions = [inquirer.List('model', message="Select the policy model to evaluate", choices=model_choices)]
            answers.update(inquirer.prompt(questions) or {})
        
        if not args.dataset_pkl:
            pkl_choices = find_pkl_files(paths.PROCESSED_DATA_DIR)
            if not pkl_choices: 
                logging.error(f"No .pkl datasets found in {paths.PROCESSED_DATA_DIR}. Exiting.")
                return
            questions = [inquirer.List('dataset_pkl', message="Select the dataset for evaluation", choices=pkl_choices)]
            answers.update(inquirer.prompt(questions) or {})
        
        if not args.rollout:
             questions = [inquirer.Confirm('rollout', message="Run closed-loop rollout evaluation?", default=False)]
             answers.update(inquirer.prompt(questions) or {})

        if not answers: 
            logging.info("No selection made. Exiting.")
            return

        # --- 2. Combine args and answers ---
        final_args = argparse.Namespace(**vars(args))
        for key, value in answers.items():
            if getattr(final_args, key) is None or getattr(final_args, key) is False:
                setattr(final_args, key, value)
        
        if not final_args.model or not final_args.dataset_pkl:
            logging.error("A model and dataset must be selected. Exiting.")
            return

        # --- 3. Load and Run Evaluation ---
        device = "cuda" if torch.cuda.is_available() else "cpu"
        
        model_path = paths.WORKSPACE_ROOT / final_args.model
        dataset_path = paths.WORKSPACE_ROOT / final_args.dataset_pkl
        
        checkpoint = torch.load(model_path, map_location=device, weights_only=False)
        frame_stack_k = checkpoint.get("frame_stack", 1)
        model_type = checkpoint["model_type"]
        is_arm_only = checkpoint.get("arm_only", False)
        num_arm_joints = checkpoint.get("num_arm_joints", checkpoint["output_dim"])
        
        # --- THE CRITICAL FIX: Read hyperparameters from the checkpoint ---
        model_hyperparams = checkpoint.get("model_hyperparams", {})
        logging.info(f"Loaded model hyperparameters: {model_hyperparams}")
        logging.info(f"Model was trained with K={frame_stack_k}, type='{model_type}'. Evaluating accordingly.")
        if is_arm_only:
            logging.info(f"Evaluating an ARM-ONLY model with {num_arm_joints} joints.")

        # --- Automatically infer the MLP width from the checkpoint file ---
        inferred_width = None
        if model_type == 'mlp':
            if 'net.0.bias' in checkpoint['state_dict']:
                inferred_width = checkpoint['state_dict']['net.0.bias'].shape[0]
                logging.info(f"Inferred MLP width from checkpoint: {inferred_width}")
            else:
                logging.error("Could not infer MLP width from checkpoint.")
                return
        
        model = build_model(
            model_type, 
            checkpoint["input_dim"], 
            checkpoint["output_dim"],
            # Pass all saved hyperparameters. The build_model function
            # will only use the ones it needs (e.g., 'width' for MLP).
            **model_hyperparams 
        ).to(device)
        model.load_state_dict(checkpoint["state_dict"])
        model.eval()
        
        norm_stats = (torch.tensor(checkpoint["X_mean"], device=device), torch.tensor(checkpoint["X_std"], device=device),
                      torch.tensor(checkpoint["y_mean"], device=device), torch.tensor(checkpoint["y_std"], device=device))
        joint_dim = checkpoint["output_dim"]
        
        logging.info(f"Loaded model and normalization stats from: {final_args.model}")

        with open(dataset_path, "rb") as f: all_trajectories = pickle.load(f)
        random.seed(final_args.seed); random.shuffle(all_trajectories)
        split_index = int(len(all_trajectories) * final_args.split_ratio)
        val_trajectories = all_trajectories[split_index:]

        # --- NEW: Slice the validation data if it's an arm-only model ---
        if is_arm_only:
            for traj in val_trajectories:
                # traj['joints_t'] = traj['joints_t'][:, :num_arm_joints]
                traj['delta_q'] = traj['delta_q'][:, :num_arm_joints]
                # if 'goal_t' in traj:
                #     # Reconstruct the correct sliced goal dimension
                #     goal_state_dim = traj['tactile_t'].shape[1] + traj['visual_t'].shape[1] + num_arm_joints
                #     traj['goal_t'] = traj['goal_t'][:, :goal_state_dim]

        output_dir = paths.MODELS_DIR / "debug"
        output_dir.mkdir(parents=True, exist_ok=True)
        logging.info(f"Loaded {len(val_trajectories)} validation trajectories from dataset.")
        logging.info(f"Evaluation outputs will be saved to: {output_dir}")
        
        if final_args.rollout:
            # --- ROLLOUT MODE ---
            all_metrics, all_preds, all_gts = [], [], []
            rollout_plot_dir = output_dir / "rollouts"
            rollout_plot_dir.mkdir(exist_ok=True)
            logging.info(f"Starting per-trajectory closed-loop rollout evaluation...")

            # joint_limits = (torch.full((joint_dim,), -2*np.pi, device=device), torch.full((joint_dim,), 2*np.pi, device=device))
            # Joint limits must always match the FULL robot's dimensionality (23),
            # not the model's output dimensionality (7).
            FULL_ROBOT_JOINT_DIM = 23 # Use a constant for clarity
            joint_limits = (
                torch.full((FULL_ROBOT_JOINT_DIM,), -2*np.pi, device=device),
                torch.full((FULL_ROBOT_JOINT_DIM,), 2*np.pi, device=device)
            )

            for i, trajectory in enumerate(val_trajectories):
                pred_np, gt_np, metrics = perform_rollout(model, trajectory, final_args.horizon, norm_stats, joint_limits, frame_stack_k, is_arm_only, num_arm_joints, device)
                all_metrics.append(metrics); all_preds.append(pred_np); all_gts.append(gt_np)
                logging.info(f"  Trajectory {i+1}/{len(val_trajectories)} | R²={metrics['r2']:.4f}")

                # Plot individual arm trajectories
                fig_arm, ax_arm = plt.subplots(figsize=(12, 6))
                num_arm_joints = min(7, joint_dim)
                for j in range(num_arm_joints):
                    ax_arm.plot(gt_np[:, j], color=f'C{j}', linestyle='--', label=f'GT Joint {j}')
                    ax_arm.plot(pred_np[:, j], color=f'C{j}', label=f'Pred Joint {j}')
                ax_arm.set_title(f'Rollout vs. GT - Arm Joints (Trajectory {i+1})')
                ax_arm.set_xlabel('Timestep'); ax_arm.set_ylabel('Joint Angle (rad)')
                ax_arm.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0)); ax_arm.grid(True, linestyle='--')
                plt.tight_layout(); plt.savefig(rollout_plot_dir / f"rollout_traj_{i+1}_arm.png"); plt.close(fig_arm)
                logging.info(f"    - Individual arm trajectory plot saved.")

                if joint_dim > num_arm_joints:
                    fig_hand, ax_hand = plt.subplots(figsize=(12, 6))
                    for j in range(num_arm_joints, joint_dim):
                        # Use a color cycle that restarts for clarity
                        color_idx = j - num_arm_joints
                        ax_hand.plot(gt_np[:, j], color=f'C{color_idx}', linestyle='--', label=f'GT Joint {j}')
                        ax_hand.plot(pred_np[:, j], color=f'C{color_idx}', label=f'Pred Joint {j}')
                    ax_hand.set_title(f'Rollout vs. GT - Hand Joints (Trajectory {i+1})')
                    ax_hand.set_xlabel('Timestep'); ax_hand.set_ylabel('Joint Angle (rad)')
                    ax_hand.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0)); ax_hand.grid(True, linestyle='--')
                    plt.tight_layout(); plt.savefig(rollout_plot_dir / f"rollout_traj_{i+1}_hand.png"); plt.close(fig_hand)
                    logging.info(f"    - Individual hand trajectory plot saved.")
                
            if all_preds:
                # Plot drift over time
                drift_per_step = np.sqrt(np.mean((all_preds[0] - all_gts[0]) ** 2, axis=1))
                plt.figure(figsize=(10, 5))
                plt.plot(drift_per_step)
                plt.xlabel("Timestep in Rollout"); plt.ylabel("Root Mean Square Error (RMSE)")
                plt.title("Prediction Drift Over Time (Sample Trajectory 1)"); plt.grid(True)
                plt.savefig(output_dir / "rollout_drift_over_time.png"); plt.close()

                # Plot per-joint error
                all_preds_np = np.concatenate(all_preds, axis=0)
                all_gts_np = np.concatenate(all_gts, axis=0)
                per_joint_error = np.sqrt(np.mean((all_preds_np - all_gts_np) ** 2, axis=0))
                plt.figure(figsize=(12, 6))
                plt.bar(np.arange(len(per_joint_error)), per_joint_error)
                plt.xlabel("Joint Index"); plt.ylabel("Overall RMSE")
                plt.title("Per-Joint Rollout Error (Averaged Across All Validation Rollouts)"); plt.grid(True)
                plt.savefig(output_dir / "rollout_per_joint_error.png"); plt.close()
                logging.info(f"Aggregate rollout plots saved in: {output_dir}")

            avg_metrics = {key: np.mean([m[key] for m in all_metrics]) for key in all_metrics[0]}
            logging.info("\n" + "="*50 + "\n          📊 AVERAGE ROLLOUT METRICS 📊\n" + "="*50)
            logging.info(f"Average MSE: {avg_metrics['mse']:.6f}\nAverage MAE: {avg_metrics['mae']:.6f}\nAverage R² : {avg_metrics['r2']:.4f}")
            logging.info(f"Individual trajectory plots saved in: {rollout_plot_dir}")

        else:
            # --- ONE-STEP MODE ---
            logging.info("Starting one-step evaluation with frame stacking...")
            is_sequence_model = model_type in ["lstm", "gru"]
            flatten_data = not is_sequence_model
            
            X_val_list, y_val_list = [], []
            for traj in val_trajectories:
                X_unstacked = np.concatenate([
                    traj['tactile_t'], traj['visual_t'], traj['joints_t'], traj['goal_t']
                ], axis=1)
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
            
            # --- Plotting ---
            joints_to_plot = {'Arm Base': 0, 'Arm Elbow': 4, 'Arm Wrist': 6, 'Hand Finger': 20}
            fig, axes = plt.subplots(2, 2, figsize=(12, 12))
            fig.suptitle('One-Step Action Prediction (Δq) vs. Ground Truth', fontsize=16)
            axes = axes.flatten()

            for i, (name, joint_idx) in enumerate(joints_to_plot.items()):
                if joint_idx >= joint_dim: continue
                ax = axes[i]
                ax.scatter(delta_q_true_np[:, joint_idx], delta_q_pred_np[:, joint_idx], alpha=0.1)
                mn, mx = delta_q_true_np[:, joint_idx].min(), delta_q_true_np[:, joint_idx].max()
                ax.plot([mn, mx], [mn, mx], "r--", label='Perfect Prediction')
                ax.set_title(f'Joint {joint_idx} ({name})')
                ax.set_xlabel('Ground Truth Action (Δq)'); ax.set_ylabel('Predicted Action (Δq)')
                ax.grid(True); ax.legend()

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plt.savefig(output_dir / "onestep_eval_scatter_multi.png"); plt.close()

            errors = delta_q_pred_np - delta_q_true_np
            plt.figure(figsize=(10, 5)); plt.hist(errors.flatten(), bins=100)
            plt.xlabel("Action Prediction Error (Δq, in rad)"); plt.ylabel("Frequency"); plt.title("One-Step Prediction Error Distribution")
            plt.grid(True); plt.tight_layout()
            plt.savefig(output_dir / "onestep_eval_error_hist.png"); plt.close()
            
            logging.info(f"Evaluation plots saved in: {output_dir}")

    except (KeyboardInterrupt, TypeError, RuntimeError) as e:
        logging.error(f"❌ An error occurred during evaluation: {e}", exc_info=True)
        logging.info("\nEvaluation cancelled or failed.")
        return
    
if __name__ == "__main__":
    main()