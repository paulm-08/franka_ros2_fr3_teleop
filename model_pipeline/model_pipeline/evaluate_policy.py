#!/usr/bin/env python3
import numpy as np
import torch
import matplotlib.pyplot as plt
import logging, os, argparse, pickle, random
from pathlib import Path
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import inquirer
import yaml
import pinocchio as pin
import tempfile

from model_pipeline.train import build_model, MLPPolicy
from model_pipeline import paths
from model_pipeline.kinematics import KinematicsSolver, get_urdf_string_from_xacro

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

def perform_rollout(model, trajectory, horizon, norm_stats, joint_limits, frame_stack_k,
                    is_arm_only, num_arm_joints, device, control_mode='joint_space'):
    """
    Closed-loop rollout that:
      - uses ground-truth sensory channels each timestep (tactile, 3D-tactile, visual),
      - uses model predictions to propagate proprioception (arm + optional hand),
      - matches the dataset feature order: [24 tactile | 23 proprio(arm+hand) | 3D-tactile | visual].
    Returns:
      pred_np: (rollout_steps, proprio_dim) predicted absolute proprio values
      gt_np:   (rollout_steps, proprio_dim) ground-truth absolute proprio values
      metrics: dict with mse, mae, r2 (computed on arm-only or all proprio as requested)
    """
    model.eval()

    # Unpack and ensure tensors on device
    X_mean_t, X_std_t, y_mean_t, y_std_t = norm_stats
    # Convert to tensors on device (they may already be tensors)
    X_mean = torch.as_tensor(X_mean_t, dtype=torch.float32, device=device)
    X_std  = torch.as_tensor(X_std_t, dtype=torch.float32, device=device)
    y_mean = torch.as_tensor(y_mean_t, dtype=torch.float32, device=device)
    y_std  = torch.as_tensor(y_std_t, dtype=torch.float32, device=device)

    joint_min, joint_max = joint_limits

    # Load trajectory arrays as tensors on device
    state_t = torch.as_tensor(trajectory["state_t"], dtype=torch.float32, device=device)
    goal_t  = torch.as_tensor(trajectory["goal_t"][0], dtype=torch.float32, device=device)  # single goal
    # ground-truth proprio is inside state_t (we will slice it out for gt)
    total_steps = state_t.shape[0]
    state_dim = state_t.shape[1]

    # default horizon handling
    if horizon is None:
        horizon = total_steps

    # constants per your spec
    tactile_dim = 24
    proprio_dim = 7 if is_arm_only else 23  # 7 arm + 16 hand
    proprio_start = tactile_dim
    proprio_end = proprio_start + proprio_dim

    # Sanity check
    if state_dim <= proprio_end:
        raise RuntimeError(f"State vector too short ({state_dim}) to contain tactile+proprio ({proprio_end}).")

    # Indices of sensory (non-proprio) features:
    # sensory = [0 : tactile_dim] + [proprio_end : state_dim]  (this includes 3D tactile and visual)
    sensory_idx_left = torch.arange(0, tactile_dim, device=device, dtype=torch.long)
    sensory_idx_right = torch.arange(proprio_end, state_dim, device=device, dtype=torch.long)
    sensory_indices = torch.cat([sensory_idx_left, sensory_idx_right], dim=0)

    # Number of rollout steps we can produce (warm-start with K frames)
    rollout_steps = min(horizon, total_steps - frame_stack_k)
    if rollout_steps <= 0:
        logging.warning("Rollout horizon too short — returning empty trajectory.")
        return None, None, None

    # --- Initialize q_pred_history with GT proprio from the first K frames (absolute values) ---
    # q_pred_history shape: (K, proprio_dim)
    q_pred_history = state_t[:frame_stack_k, proprio_start:proprio_end].clone()

    predicted_q_trajectory = []  # will collect absolute predicted proprio per step

    # Detect whether model expects sequence input or flattened MLP input
    model_name = model.__class__.__name__.lower()
    is_sequence_model = any(k in model_name for k in ['lstm', 'gru', 'transformer', 'rnn'])

    # Main rollout loop
    for i in range(rollout_steps):
        t = i + frame_stack_k - 1

        # Build sensory stack: shape (K, sensory_dim)
        sensory_stack = state_t[t - frame_stack_k + 1 : t + 1, sensory_indices]

        # Build state_window by concatenating sensory_stack and q_pred_history
        # sensory_stack: (K, sensory_dim)
        # q_pred_history: (K, proprio_dim)
        state_window = torch.cat([sensory_stack, q_pred_history], dim=1)  # (K, sensory_dim + proprio_dim) == (K, state_dim)

        # Build goal window (repeat same goal K times). goal_t has shape (state_dim,)
        goal_window = goal_t.unsqueeze(0).repeat(frame_stack_k, 1)  # (K, state_dim)

        # Full state sequence: concat(state_window, goal_window) -> shape (K, 2*state_dim)
        full_state_sequence = torch.cat([state_window, goal_window], dim=1)  # (K, 2*state_dim)

        # --- Normalize exactly like training ---
        # X_mean and X_std are expected to be 1D vectors of length D = 2*state_dim (state + goal).
        if is_sequence_model:
            # full_state_sequence shape: (K, D)
            # X_mean shape: (D,) -> broadcast to (K, D)
            state_norm = (full_state_sequence - X_mean) / (X_std + 1e-8)
            x_tensor = state_norm.unsqueeze(0)  # [B=1, T=K, D]
        else:
            # flatten to (K*D,)
            x_flat = full_state_sequence.flatten()
            # X_mean.repeat(K) gives length K*D if X_mean has length D
            x_mean_flat = X_mean.repeat(frame_stack_k)
            x_std_flat  = X_std.repeat(frame_stack_k)
            state_norm_flat = (x_flat - x_mean_flat) / (x_std_flat + 1e-8)
            x_tensor = state_norm_flat.unsqueeze(0)  # [1, K*D]

        # --- Model forward (on device) ---
        with torch.no_grad():
            y_pred_norm = model(x_tensor).squeeze(0)  # shape: (action_dim,) or (proprio_dim,) depending on model
            # denormalize
            y_pred = (y_pred_norm * y_std) + y_mean  # all tensors on device

        # y_pred may be longer than proprio_dim (e.g. full action -> arm+hand). We will take the first proprio_dim entries
        # Ensure y_pred is same device and shape
        y_pred = y_pred.cpu()
        # slice to proprio length
        delta_proprio = y_pred[:proprio_dim].numpy()  # numpy array length proprio_dim

        # Reconstruct next absolute proprio (last predicted q + delta)
        last_q = q_pred_history[-1].cpu().numpy()
        q_next = last_q + delta_proprio  # numpy

        # Clip using joint limits (joint_min/joint_max may be full-robot; slice first proprio_dim)
        jm = joint_min.cpu().numpy()[:proprio_dim]
        jM = joint_max.cpu().numpy()[:proprio_dim]
        q_next = np.minimum(np.maximum(q_next, jm), jM)

        predicted_q_trajectory.append(q_next.astype(np.float32))

        # roll q_pred_history and insert q_next (as tensor)
        q_next_tensor = torch.as_tensor(q_next, dtype=torch.float32, device=device)
        q_pred_history = torch.roll(q_pred_history, shifts=-1, dims=0)
        q_pred_history[-1] = q_next_tensor

    # Convert predicted trajectory to numpy (shape: rollout_steps x proprio_dim)
    pred_np = np.array(predicted_q_trajectory)  # absolute proprio predictions

    # Ground-truth absolute proprio for the same timesteps:
    # The GT proprio frames we compare to are taken from state_t starting at index frame_stack_k
    gt_proprio_all = state_t[frame_stack_k : frame_stack_k + rollout_steps, proprio_start:proprio_end].cpu().numpy()

    gt_np = gt_proprio_all

    # Build metrics: if arm-only, compute only first num_arm_joints columns
    if is_arm_only:
        gt_for_metrics = gt_np[:, :num_arm_joints]
        pred_for_metrics = pred_np[:, :num_arm_joints]
        logging.info(f"  (Metrics on {num_arm_joints}-DOF arm only)")
    else:
        gt_for_metrics = gt_np
        pred_for_metrics = pred_np

    metrics = {
        "mse": mean_squared_error(gt_for_metrics, pred_for_metrics),
        "mae": mean_absolute_error(gt_for_metrics, pred_for_metrics),
        "r2": r2_score(gt_for_metrics, pred_for_metrics),
    }

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
        control_mode = checkpoint.get("control_mode","joint_space")
        frame_stack_k = checkpoint.get("frame_stack", 1)
        model_type = checkpoint["model_type"]
        is_arm_only = checkpoint.get("arm_only", False)
        num_arm_joints = checkpoint.get("num_arm_joints", checkpoint["output_dim"])
        num_arm_joints_action = checkpoint["output_dim"] - 16 if is_arm_only else checkpoint["output_dim"]
        validation = checkpoint.get("validation", False)
        if "training_config" not in checkpoint:
            logging.error("Model checkpoint does not contain a config. Using default config.")
            config_path_abs = paths.WORKSPACE_ROOT / "config/config.yaml"
            with open(config_path_abs, 'r') as f:
                config = yaml.safe_load(f)
        else:
            config = checkpoint["training_config"]
        
        # --- Read hyperparameters from the checkpoint ---
        model_hyperparams = checkpoint.get("model_hyperparams", {})
        logging.info(f"Loaded model hyperparameters: {model_hyperparams}")
        logging.info(f"Model was trained in {control_mode}, with K={frame_stack_k}, type='{model_type}' and {'no' if validation else ''}validation loss. Evaluating accordingly.")
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

        solver=None
        if control_mode=='task_space':
            # --- Initialize KinematicsSolver for task-space rollouts ---
            urdf_content = get_urdf_string_from_xacro()
            if not urdf_content: 
                logging.error("Failed to generate URDF, cannot proceed."); return

            urdf_temp_file = None
            try:
                # 2b. Save to a temporary file for Pinocchio to load
                # This creates a file with a unique name in the system's temp directory
                with tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.urdf', encoding='utf-8') as f:
                    f.write(urdf_content)
                    urdf_temp_file = Path(f.name)
            
                kinematics_config = config.get("kinematics", {})
                solver = KinematicsSolver(
                    urdf_content=urdf_temp_file,
                    end_effector_frame_name=kinematics_config.get("ee_frame", "fr3_hand_tcp"),
                    tactile_frame_names=kinematics_config.get("tactile_frames", []),
                    visualize=False,
                )
            except Exception as e:
                logging.error(f"Error initializing KinematicsSolver: {e}")
                return
            finally:
                # Clean up the temporary URDF file
                if urdf_temp_file and urdf_temp_file.exists():
                    urdf_temp_file.unlink()

        # --- Load trajectories
        with open(dataset_path, "rb") as f: all_trajectories = pickle.load(f)
        random.seed(final_args.seed); random.shuffle(all_trajectories)

        if validation:
            split_index = int(len(all_trajectories) * final_args.split_ratio)
            val_trajectories = all_trajectories[split_index:]
        else:
            # Random or all trajectories
            val_trajectories = all_trajectories

        # --- NEW: Slice the validation data if it's an arm-only model ---
        if is_arm_only:
            for traj in val_trajectories:
                # traj['joints_t'] = traj['joints_t'][:, :num_arm_joints]
                if control_mode == "joint_space":
                    traj['action_t'] = traj['action_t'][:, :num_arm_joints]
                # if 'goal_t' in traj:
                #     # Reconstruct the correct sliced goal dimension
                #     goal_state_dim = traj['tactile_t'].shape[1] + traj['visual_t'].shape[1] + num_arm_joints
                #     traj['goal_t'] = traj['goal_t'][:, :goal_state_dim]
                else:
                    traj['action_t'] = traj['action_t'][:, :6]

        output_dir = paths.MODELS_DIR / "debug"
        output_dir.mkdir(parents=True, exist_ok=True)
        logging.info(f"Loaded {len(val_trajectories)} validation trajectories from dataset.")
        logging.info(f"Evaluation outputs will be saved to: {output_dir}")
        
        if final_args.rollout:
            # --- ROLLOUT MODE ---
            all_metrics, all_preds, all_gts = [], [], []
            rollout_plot_dir = output_dir / "rollouts"
            rollout_plot_dir.mkdir(exist_ok=True)
            logging.info("Starting per-trajectory closed-loop rollout evaluation...")

            FULL_ROBOT_JOINT_DIM = 23
            joint_limits = (
                torch.full((FULL_ROBOT_JOINT_DIM,), -2*np.pi, device=device),
                torch.full((FULL_ROBOT_JOINT_DIM,),  2*np.pi, device=device)
            )

            num_arm_joints = 7
            num_arm_task_dof = 6
            num_hand_joints = 16

            # ----- determine DOFs to plot -----
            if control_mode == "joint_space":
                arm_dim = num_arm_joints
                arm_label = "Joint Angle (rad)"
                arm_names = [f"Joint {i+1}" for i in range(arm_dim)]
            else:
                arm_dim = num_arm_task_dof
                arm_label = "Task Δ (m / rad)"
                arm_names = ["Δx","Δy","Δz","Δroll","Δpitch","Δyaw"]

            for i, traj in enumerate(val_trajectories):
                pred_np, gt_np, metrics = perform_rollout(
                    model=model,
                    trajectory=traj,
                    horizon=None,
                    norm_stats=norm_stats,
                    joint_limits=joint_limits,
                    frame_stack_k=frame_stack_k,
                    is_arm_only=is_arm_only,
                    num_arm_joints=7,
                    device=device,
                    control_mode=config.get("control_mode", "joint_space"),
                )
                all_metrics.append(metrics)
                all_preds.append(pred_np)
                all_gts.append(gt_np)
                logging.info(f"  Trajectory {i+1}/{len(val_trajectories)} | R²={metrics['r2']:.4f}")

                if i >= 10:       # only plot first 10
                    continue

                # ---------- Arm ----------
                fig, ax = plt.subplots(figsize=(12,6))
                for j in range(arm_dim):
                    c = f"C{j%10}"
                    ax.plot(gt_np[:, j], c=c, ls='--', label=f"GT {arm_names[j]}")
                    ax.plot(pred_np[:, j], c=c, label=f"Pred {arm_names[j]}")
                ax.set(title=f"Rollout vs GT – Arm DOFs (Traj {i+1})",
                    xlabel="Timestep", ylabel=arm_label)
                ax.legend(loc="upper right", bbox_to_anchor=(1.15,1))
                ax.grid(ls='--')
                plt.tight_layout()
                plt.savefig(rollout_plot_dir/f"rollout_traj_{i+1}_arm.png")
                plt.close(fig)

                # ---------- Hand ----------
                if not is_arm_only and gt_np.shape[1] > arm_dim:
                    fig, ax = plt.subplots(figsize=(12,6))
                    hand_start = arm_dim
                    hand_to_plot = [0,3,7,11,15]   # sample subset
                    for k,hj in enumerate(hand_to_plot):
                        idx = hand_start + hj
                        if idx >= gt_np.shape[1]: break
                        c = f"C{k%10}"
                        ax.plot(gt_np[:,idx], c=c, ls='--', label=f"GT Hand J{hj+1}")
                        ax.plot(pred_np[:,idx], c=c, label=f"Pred Hand J{hj+1}")
                    ax.set(title=f"Rollout vs GT – Hand DOFs (Traj {i+1})",
                        xlabel="Timestep", ylabel="Joint Angle (rad)")
                    ax.legend(loc="upper right", bbox_to_anchor=(1.15,1))
                    ax.grid(ls='--')
                    plt.tight_layout()
                    plt.savefig(rollout_plot_dir/f"rollout_traj_{i+1}_hand.png")
                    plt.close(fig)

            # ---------- Aggregate ----------
            if all_preds:
                drift = np.sqrt(np.mean((all_preds[0]-all_gts[0])**2, axis=1))
                plt.figure(figsize=(10,5))
                plt.plot(drift)
                plt.xlabel("Timestep"); plt.ylabel("RMSE")
                plt.title("Prediction Drift Over Time (Traj 1)")
                plt.grid(True)
                plt.savefig(output_dir/"rollout_drift_over_time.png")
                plt.close()

                all_pred = np.concatenate(all_preds)
                all_gt   = np.concatenate(all_gts)
                rmse_per_dof = np.sqrt(np.mean((all_pred-all_gt)**2, axis=0))
                plt.figure(figsize=(12,6))
                plt.bar(np.arange(len(rmse_per_dof)), rmse_per_dof)
                plt.xlabel("DOF Index"); plt.ylabel("Overall RMSE")
                plt.title("Per-DOF Rollout Error (All Validation Rollouts)")
                plt.grid(True)
                plt.tight_layout()
                plt.savefig(output_dir/"rollout_per_dof_error.png")
                plt.close()

            avg = {k: np.mean([m[k] for m in all_metrics]) for k in all_metrics[0]}
            logging.info("\n"+"="*50+"\n       📊 AVERAGE ROLLOUT METRICS 📊\n"+"="*50)
            logging.info(f"Avg MSE: {avg['mse']:.6f}\nAvg MAE: {avg['mae']:.6f}\nAvg R² : {avg['r2']:.4f}")
            logging.info(f"Plots saved in {rollout_plot_dir}")

        else:
            # --- ONE-STEP MODE ---
            logging.info("Starting one-step evaluation with frame stacking...")
            is_sequence_model = model_type in ["lstm", "gru"]
            flatten_data = not is_sequence_model
            
            X_val_list, y_val_list = [], []
            for traj in val_trajectories:
                X_unstacked = np.concatenate([
                    traj['state_t'], traj['goal_t']
                ], axis=1)
                y_unstacked = traj['action_t']
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
                action_pred = (pred_norm * y_std) + y_mean
            
            action_pred_np = to_np(action_pred)
            action_true_np = to_np(y_val_true)
            
            metrics = {'mse': mean_squared_error(action_true_np, action_pred_np), 
                       'mae': mean_absolute_error(action_true_np, action_pred_np), 
                       'r2': r2_score(action_true_np, action_pred_np)}

            logging.info("\n" + "="*50 + "\n          📊 ONE-STEP (action) METRICS 📊\n" + "="*50)
            logging.info(f"MSE: {metrics['mse']:.6f}\nMAE: {metrics['mae']:.6f}\nR² : {metrics['r2']:.4f}")
            
            # --- Plotting: Adaptive to control mode and arm_only flag ---
        num_arm_joints = 7        # Franka arm
        num_arm_task_dof = 6      # (x, y, z, roll, pitch, yaw)
        num_hand_joints = 16

        if control_mode == "joint_space":
            arm_dim = num_arm_joints
            arm_label = "Δq (rad)"
            arm_names = [f"Joint {i+1}" for i in range(arm_dim)]
        else:
            arm_dim = num_arm_task_dof
            arm_label = "Δx (m / rad)"
            arm_names = ["Δx", "Δy", "Δz", "Δroll", "Δpitch", "Δyaw"]

        # --- Build list of plotted DOFs ---
        plot_indices = []
        plot_names = []

        # → Always plot all arm DOFs
        for i in range(arm_dim):
            plot_indices.append(i)
            plot_names.append(arm_names[i])

        # → If not arm-only, plot a few representative hand joints
        if not is_arm_only:
            hand_start = arm_dim
            hand_to_plot = [0, 3, 7, 11, 15]  # sample of 5 joints
            for hj in hand_to_plot:
                if hand_start + hj < action_true_np.shape[1]:
                    plot_indices.append(hand_start + hj)
                    plot_names.append(f"Hand J{hj+1}")

        # --- Dynamic grid layout ---
        num_plots = len(plot_indices)
        ncols = 3 if num_plots > 3 else num_plots
        nrows = int(np.ceil(num_plots / ncols))

        fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4.5 * nrows))
        axes = np.array(axes).flatten()
        fig.suptitle("One-Step Action Prediction vs. Ground Truth", fontsize=16)

        # --- Scatter plots per DOF ---
        for i, (name, idx) in enumerate(zip(plot_names, plot_indices)):
            ax = axes[i]
            if idx >= action_true_np.shape[1]:
                continue
            ax.scatter(action_true_np[:, idx], action_pred_np[:, idx], alpha=0.15, s=10)
            mn, mx = np.min(action_true_np[:, idx]), np.max(action_true_np[:, idx])
            ax.plot([mn, mx], [mn, mx], "r--", linewidth=1, label="Perfect Prediction")
            ax.set_title(name)
            ax.set_xlabel(f"Ground Truth ({arm_label})")
            ax.set_ylabel(f"Predicted ({arm_label})")
            ax.grid(True)
            ax.legend()

        # Hide unused subplots (if any)
        for j in range(i + 1, len(axes)):
            axes[j].axis("off")

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig(output_dir / "onestep_eval_scatter_all.png")
        plt.close()

        # --- Error histogram ---
        errors = action_pred_np - action_true_np
        plt.figure(figsize=(10, 5))
        plt.hist(errors.flatten(), bins=100, color="steelblue", alpha=0.8)
        plt.xlabel(f"Action Prediction Error ({arm_label})")
        plt.ylabel("Frequency")
        plt.title("One-Step Prediction Error Distribution")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(output_dir / "onestep_eval_error_hist.png")
        plt.close()

        logging.info(f"Evaluation plots saved in: {output_dir}")

    except (KeyboardInterrupt, TypeError, RuntimeError) as e:
        logging.error(f"❌ An error occurred during evaluation: {e}", exc_info=True)
        logging.info("\nEvaluation cancelled or failed.")
        return
    
if __name__ == "__main__":
    main()