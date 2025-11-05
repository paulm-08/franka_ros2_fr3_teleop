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
import math
from scipy.spatial.transform import Rotation as R

from model_pipeline.train import build_model, MLPPolicy
from model_pipeline import paths
from model_pipeline.utils import find_policy_models, find_pkl_files

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def to_np(tensor):
    return tensor.detach().cpu().numpy()

def normalize_quaternion(q):
    return q / np.linalg.norm(q)

def quaternion_from_rpy_delta(rpy):
    """
    Converts a small Roll-Pitch-Yaw (XYZ) angular delta into a unit quaternion.
    Used to convert the 3D angular component of the predicted 6D task delta
    into a quaternion for multiplication.
    """
    roll, pitch, yaw = rpy
    
    # For small angles, we can approximate the quaternion components (sin(theta/2) ~ theta/2)
    # This assumes the model predicts the *change in orientation* as an angular velocity vector.
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    
    # Simple normalization to ensure it's a unit quaternion
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return np.array([qx/norm, qy/norm, qz/norm, qw/norm], dtype=np.float32)

def quaternion_multiply(q1, q2):
    """
    Multiplies two quaternions q_new = q1 * q2, where q1 is current pose, q2 is delta.
    Expects q = [x, y, z, w].
    
    This propagation is crucial for applying the predicted rotation delta.
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    
    # Simple normalization after multiplication
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    return np.array([x/norm, y/norm, z/norm, w/norm], dtype=np.float32)

def perform_rollout(model, trajectory, horizon, norm_stats, joint_limits, frame_stack_k,
                    is_arm_only, num_arm_joints, device, control_mode='joint_space', use_goal=False):
    """
    Closed-loop rollout with corrected T|P|V feature alignment and task-space pose propagation.
    The Proprioception (P) dimension now represents 7D EE Pose + 16D Hand Joints.
    """
    model.eval()
    
    # --- Normalization tensors ---
    X_mean, X_std, y_mean, y_std = [
        torch.as_tensor(t, dtype=torch.float32, device=device) for t in norm_stats
    ]
    joint_min, joint_max = [
        torch.as_tensor(j, dtype=torch.float32, device=device) for j in joint_limits
    ]

    # --- Load FULL trajectory data ---
    state_t = torch.as_tensor(trajectory["state_t"], dtype=torch.float32, device=device)
    goal_t = torch.as_tensor(trajectory["goal_t"][0], dtype=torch.float32, device=device)
    total_steps, state_dim = state_t.shape

    # --- DEFINE STATE DIMENSIONS (Structure in GT data is T | P | V) ---
    TACTILE_DIM = 24
    EE_POSE_DIM = 7  # [x, y, z, qx, qy, qz, qw]
    HAND_JOINT_DIM = 16
    FULL_PROP_DIM = EE_POSE_DIM + HAND_JOINT_DIM # 23 Total

    # P start/end indices are constant regardless of control mode
    PROP_START_IDX = TACTILE_DIM 
    PROP_END_IDX = PROP_START_IDX + FULL_PROP_DIM # 24 + 23 = 47
    VISUAL_START_IDX = PROP_END_IDX
    
    rollout_steps = min(horizon or total_steps, total_steps - frame_stack_k)
    
    # Initialize proprioception history (q_pred_history) with ground-truth data
    # q_pred_history now contains EE_POSE + HAND_JOINTS
    q_pred_history = state_t[:frame_stack_k, PROP_START_IDX:PROP_END_IDX].clone()
    
    predicted_q_trajectory = []
    is_sequence_model = any(k in model.__class__.__name__.lower() for k in ['lstm', 'gru', 'transformer', 'rnn'])

    # logging.info(f"[LOG_MAG] Y_MEAN (Action Delta): {y_mean.shape}")
    
    for i in range(rollout_steps):
        t = i + frame_stack_k - 1

        # 1. Extract Ground Truth Sensory data
        tactile_gt_stack = state_t[t - frame_stack_k + 1 : t + 1, 0:TACTILE_DIM]
        visual_gt_stack = state_t[t - frame_stack_k + 1 : t + 1, VISUAL_START_IDX:state_dim]
        
        # --- CRITICAL FIX: RECONSTRUCT STATE AS [T | P_pred | V] ---
        # P_pred is [EE_POSE (7D), HAND_JOINTS (16D)]
        state_window = torch.cat([
            tactile_gt_stack,
            q_pred_history, 
            visual_gt_stack 
        ], dim=1) 
        
        # Add goal
        goal_window = goal_t.unsqueeze(0).repeat(frame_stack_k, 1)
        if use_goal:
            full_state_sequence = torch.cat([state_window, goal_window], dim=1)
        else:
            full_state_sequence = state_window

        # Apply normalization (X_mean/X_std)
        if is_sequence_model:
            x_tensor = ((full_state_sequence - X_mean) / (X_std + 1e-8)).unsqueeze(0)
        else:
            flat = full_state_sequence.flatten()
            norm_flat = (flat - X_mean.repeat(frame_stack_k)) / (X_std.repeat(frame_stack_k) + 1e-8)
            x_tensor = norm_flat.unsqueeze(0)

        # --- Model forward and De-normalize (Y_mean/Y_std) ---
        with torch.no_grad():
            y_pred_norm = model(x_tensor).squeeze(0)
            delta_action_pred = (y_pred_norm * y_std) + y_mean
            
        # --- Propagate next proprio ---
        # last_q_full = [EE_POSE(7), HAND_JOINTS(16)]
        last_q_full = q_pred_history[-1]
        
        if control_mode == 'joint_space':
            
            # Calculate next full state
            if is_arm_only:
                delta_q_pred_arm = delta_action_pred
                hand_zeros = torch.zeros(FULL_PROP_DIM - num_arm_joints, device=device)
                full_delta_q = torch.cat([delta_q_pred_arm, hand_zeros])
                # logging.info(f"[STEP {i:03d}] ΔQ_ARM | Min:{delta_q_pred_arm.min().item():.6f}, Max:{delta_q_pred_arm.max().item():.6f}, Norm:{delta_q_pred_arm.norm().item():.6f}")

            else:
                full_delta_q = delta_action_pred
            
            # --- Joint Limit Clamping (Only applies to Hand Joints) ---
            q_pred_next = last_q_full + full_delta_q
            q_pred_next[:EE_POSE_DIM] = torch.clamp(
                q_pred_next[:EE_POSE_DIM], 
                joint_min[:EE_POSE_DIM], 
                joint_max[:EE_POSE_DIM]
            )


        elif control_mode == 'task_space':
            # ACTION: 6D EE Pose Delta (Linear+Angular) + 16D Hand Joint Delta

            # 1. Calculate the action
            if is_arm_only:
                delta_x_ee = to_np(delta_action_pred) # 6D
                delta_q_hand = torch.zeros(FULL_PROP_DIM - num_arm_joints, device=device)
                # full_delta_q = torch.cat([delta_q_pred_arm, hand_zeros])

            else:
                # Split the action into EE Delta (6D) and Hand Delta (16D)
                delta_x_ee = to_np(delta_action_pred[:6]) # 6D
                delta_q_hand = delta_action_pred[6:]      # 16D (assuming model predicts hand delta)

            # 2. Get last pose components
            last_pose_np = to_np(last_q_full[:EE_POSE_DIM])
            last_x_pos, last_q_quat = last_pose_np[:3], last_pose_np[3:]

            # 3. Propagate Position (Linear Delta)
            next_x_pos = last_x_pos + delta_x_ee[:3]
            
            # 4. Propagate Orientation (Angular Delta)
            delta_rpy = delta_x_ee[3:]
            q_delta = quaternion_from_rpy_delta(delta_rpy)
            next_q_quat = quaternion_multiply(last_q_quat, q_delta)
            
            # 5. Combine next EE Pose (7D)
            next_pose_np = np.concatenate([next_x_pos, next_q_quat])
            next_pose = torch.as_tensor(next_pose_np, dtype=torch.float32, device=device)
            
            # 6. Propagate Hand Joints (Simple addition)
            next_hand_q = last_q_full[EE_POSE_DIM:] + delta_q_hand
            
            # 7. Combine next full proprioceptive state (7D EE Pose + 16D Hand Joints)
            q_pred_next = torch.cat([next_pose, next_hand_q])
            
        else:
            raise ValueError(f"Unknown control_mode: {control_mode}")

        # --- Joint Limit Clamping (Only applies to Hand Joints) ---
        # NOTE: Arm-joint limits are irrelevant as we track EE pose, but hand joint
        # limits must still be applied to the 16D hand part of q_pred_next.
        # Assuming joint_limits is FULL_PROP_DIM long, we only clamp the 16 hand joints.
        HAND_LIMIT_START = EE_POSE_DIM
        q_pred_next[HAND_LIMIT_START:] = torch.clamp(
            q_pred_next[HAND_LIMIT_START:], 
            joint_min[HAND_LIMIT_START:], 
            joint_max[HAND_LIMIT_START:]
        )
        
        # logging.info(f"[STEP {i:03d}] P_NEXT | EE Pos Avg:{q_pred_next[:3].mean().item():.3f}, Hand Jts Min:{q_pred_next[HAND_LIMIT_START:].min().item():.3f}")

        # Update history
        predicted_q_trajectory.append(to_np(q_pred_next))
        q_pred_history = torch.roll(q_pred_history, shifts=-1, dims=0)
        q_pred_history[-1] = q_pred_next

    # --- Metrics Calculation ---
    pred_np = np.array(predicted_q_trajectory)
    gt_np = to_np(state_t[frame_stack_k : rollout_steps + frame_stack_k, PROP_START_IDX:PROP_END_IDX])

    # If is_arm_only=True, metrics are calculated on the 7D EE pose only
    if is_arm_only:
        # EE Pose is the first 7 elements of the proprioception vector
        gt_for_metrics = gt_np[:, :EE_POSE_DIM]
        pred_for_metrics = pred_np[:, :EE_POSE_DIM]
        logging.info(f"  (Metrics on 7D EE Pose only)")
    else:
        # Metrics on the full 23D vector (EE Pose + Hand Joints)
        gt_for_metrics = gt_np
        pred_for_metrics = pred_np

    metrics = {
        'mse': mean_squared_error(gt_for_metrics, pred_for_metrics),
        'mae': mean_absolute_error(gt_for_metrics, pred_for_metrics),
        'r2': r2_score(gt_for_metrics, pred_for_metrics)
    }

    # logging.info(f"[DEBUG] Final rollout metrics: {metrics}")
    
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
        logging.info(f"Model was trained in {control_mode}, with K={frame_stack_k}, type='{model_type}' and {'WITH' if validation else 'WITHOUT'} validation loss. Evaluating accordingly.")
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
            from model_pipeline.kinematics import KinematicsSolver, get_urdf_string_from_xacro
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
        
        # --- ONE-STEP MODE ---
        logging.info("Starting one-step evaluation with frame stacking...")
        is_sequence_model = model_type in ["lstm", "gru", "transformer"]
        flatten_data = not is_sequence_model
        
        X_val_list, y_val_list = [], []
        for traj in val_trajectories:
            if config.get('use_goal', False):
                X_unstacked = np.concatenate([
                    traj['state_t'], traj['goal_t']
                ], axis=1)
            else:
                X_unstacked = traj['state_t']
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
            hand_to_plot = list(range(15))
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
                arm_label = "Position (m) / Euler Angle (rad)"
                arm_names = ["x", "y", "z", "roll", "pitch", "yaw"]

            for i, traj in enumerate(val_trajectories[:50]):
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
                    use_goal=config.get("use_goal", False)
                )
                all_metrics.append(metrics)
                all_preds.append(pred_np)
                all_gts.append(gt_np)
                logging.info(f"  Trajectory {i+1}/{len(val_trajectories)} | MSE = {metrics['mse']} | R²={metrics['r2']:.4f}")

                if i >= 10:       # only plot first 10
                    continue

                # --- DATA PREPARATION FOR PLOTTING ---
                # By default, use the raw numpy arrays
                gt_to_plot = gt_np
                pred_to_plot = pred_np
                
                # If in task space, convert quaternions to Euler angles for smooth plots
                if control_mode == 'task_space':
                    # Prepare empty arrays for position + euler angles
                    gt_plot_data = np.zeros((gt_np.shape[0], 6))
                    pred_plot_data = np.zeros((pred_np.shape[0], 6))

                    # Copy over the position data (x, y, z)
                    gt_plot_data[:, :3] = gt_np[:, :3]
                    pred_plot_data[:, :3] = pred_np[:, :3]
                    
                    # --- STEP 1: Convert Quaternions to Euler Angles ---
                    # NOTE: Scipy expects quaternion as [x, y, z, w]
                    gt_euler = R.from_quat(gt_np[:, 3:7]).as_euler('xyz', degrees=False)
                    pred_euler = R.from_quat(pred_np[:, 3:7]).as_euler('xyz', degrees=False)

                    # --- STEP 2: "Unwrap" the angles to remove discontinuities ---
                    # This is the key fix. We apply it column-by-column (axis=0).
                    gt_euler_unwrapped = np.unwrap(gt_euler, axis=0)
                    pred_euler_unwrapped = np.unwrap(pred_euler, axis=0)
                    
                    # --- STEP 3: Place the continuous angles into the plot data ---
                    gt_plot_data[:, 3:] = gt_euler_unwrapped
                    pred_plot_data[:, 3:] = pred_euler_unwrapped
                    
                    # Update the variables used for plotting
                    gt_to_plot = gt_plot_data
                    pred_to_plot = pred_plot_data

                # ---------- Arm ----------
                fig, ax = plt.subplots(figsize=(12, 6))
                for j in range(arm_dim):
                    c = f"C{j%10}"
                    # UPDATED: Use the processed data for plotting
                    ax.plot(gt_to_plot[:, j], c=c, ls='--', label=f"GT {arm_names[j]}")
                    ax.plot(pred_to_plot[:, j], c=c, label=f"Pred {arm_names[j]}")
                ax.set(title=f"Rollout vs GT – Arm DOFs (Traj {i+1})",
                    xlabel="Timestep", ylabel=arm_label)
                ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1))
                ax.grid(ls='--')
                plt.tight_layout()
                plt.savefig(rollout_plot_dir / f"rollout_traj_{i+1}_arm.png")
                plt.close(fig)

                # ---------- Hand ----------
                if not is_arm_only and gt_np.shape[1] > arm_dim:
                    fig, ax = plt.subplots(figsize=(12,6))
                    hand_start = arm_dim
                    hand_to_plot = range(15)  # sample subset
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
            valid_pairs = [
                (p, g) for p, g in zip(all_preds, all_gts)
                if p is not None and g is not None and len(p) > 0 and p.shape == g.shape
            ]

            if len(valid_pairs) == 0:
                logging.warning("⚠️ No valid rollout predictions found — skipping aggregate plots.")
            else:
                logging.info(f"Generating aggregate rollout plots for {len(valid_pairs)} trajectories...")

                num_to_plot = min(3, len(valid_pairs))
                num_arm_joints = 7
                num_hand_joints = 16
                total_dofs = num_arm_joints if is_arm_only else (num_arm_joints + num_hand_joints)

                fig, axes = plt.subplots(num_to_plot, 1, figsize=(10, 4*num_to_plot), sharex=True)
                if num_to_plot == 1:
                    axes = [axes]

                for idx in range(num_to_plot):
                    pred_np, gt_np = valid_pairs[idx]

                    # --- Always slice the relevant DOFs ---
                    pred_np = pred_np[:, :total_dofs]
                    gt_np   = gt_np[:, :total_dofs]

                    if is_arm_only:
                        # --- Arm-only drift ---
                        drift_arm = np.sqrt(np.mean((pred_np - gt_np) ** 2, axis=1))
                        axes[idx].plot(drift_arm, label="Arm Drift", color="C0")

                    else:
                        # --- Separate Arm and Hand drift curves ---
                        drift_arm  = np.sqrt(np.mean(
                            (pred_np[:, :num_arm_joints] - gt_np[:, :num_arm_joints]) ** 2, axis=1
                        ))
                        drift_hand = np.sqrt(np.mean(
                            (pred_np[:, num_arm_joints:total_dofs] - gt_np[:, num_arm_joints:total_dofs]) ** 2, axis=1
                        ))

                        axes[idx].plot(drift_arm,  label="Arm Drift",  color="C0")
                        axes[idx].plot(drift_hand, label="Hand Drift", color="C1")

                    axes[idx].set_title(f"Prediction Drift Over Time – Traj {idx+1}")
                    axes[idx].set_ylabel("RMSE")
                    axes[idx].grid(ls='--')
                    axes[idx].legend()

                axes[-1].set_xlabel("Timestep")
                plt.tight_layout()
                plt.savefig(output_dir / "rollout_drift_over_time.png")
                plt.close(fig)

                # ---------- Per-DOF Rollout Error ----------
                all_pred = np.concatenate([p[:, :total_dofs] for p, _ in valid_pairs], axis=0)
                all_gt   = np.concatenate([g[:, :total_dofs] for _, g in valid_pairs], axis=0)

                rmse_per_dof = np.sqrt(np.mean((all_pred - all_gt) ** 2, axis=0))

                plt.figure(figsize=(12, 6))
                plt.bar(np.arange(total_dofs), rmse_per_dof, color='C0' if is_arm_only else 'C2')
                plt.xlabel("DOF Index")
                plt.ylabel("Overall RMSE")
                title_suffix = " (Arm-only)" if is_arm_only else " (Arm + Hand)"
                plt.title(f"Per-DOF Rollout Error{title_suffix}")
                plt.grid(True)
                plt.tight_layout()
                plt.savefig(output_dir / "rollout_per_dof_error.png")
                plt.close()
    
            avg = {k: np.mean([m[k] for m in all_metrics]) for k in all_metrics[0]}
            logging.info("\n"+"="*50+"\n       📊 AVERAGE ROLLOUT METRICS 📊\n"+"="*50)
            logging.info(f"Avg MSE: {avg['mse']:.6f}\nAvg MAE: {avg['mae']:.6f}\nAvg R² : {avg['r2']:.4f}")
            logging.info(f"Plots saved in {rollout_plot_dir}")

    except (KeyboardInterrupt, TypeError, RuntimeError) as e:
        logging.error(f"❌ An error occurred during evaluation: {e}", exc_info=True)
        logging.info("\nEvaluation cancelled or failed.")
        return
    
if __name__ == "__main__":
    main()