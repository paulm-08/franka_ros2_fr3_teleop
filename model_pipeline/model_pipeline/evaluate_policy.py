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
from torch.utils.data import TensorDataset, DataLoader

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

def quaternion_normalize(q):
    norm = np.linalg.norm(q)
    # Avoid division by zero, though unlikely if propagation is working
    return q / norm if norm > 1e-8 else np.array([0, 0, 0, 1.0])


def perform_rollout(model, trajectory, horizon, norm_stats, joint_limits, frame_stack_k,
                    is_arm_only, num_arm_joints, device, control_mode='joint_space', 
                    use_goal=False, solver=None,
                    start_time_idx=None,
                    mimic_rollout_start=False, use_average_policy=False,
                    proprio_only=False,
                    action_space='delta'):
    """
    Closed-loop rollout with added controls for start time and history initialization
    to diagnose state-mismatch problems.
    
    Args:
        start_time_idx (int, optional): The frame index to start the rollout from.
                                     If None, defaults to frame_stack_k - 1.
        mimic_rollout_start (bool, optional): If True, initializes the history
                                     by repeating the state at start_time_idx k times
                                     (simulating a "zero-velocity" start).
                                     If False, uses the ground-truth history.
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
    ARM_PROP_DIM = 7  # [x, y, z, qx, qy, qz, qw]
    ARM_POSE_6D_DIM = 6 # [x, y, z, r1, r2, r3] - Model Output Dimension for Task Space absolute
    HAND_JOINT_DIM = 16
    FULL_PROP_DIM = ARM_PROP_DIM + HAND_JOINT_DIM # 23 Total

    PROP_START_IDX = TACTILE_DIM 
    PROP_END_IDX = PROP_START_IDX + FULL_PROP_DIM # 24 + 23 = 47
    VISUAL_START_IDX = PROP_END_IDX
    
    # --- Set up start time and history ---
    if start_time_idx is None: start_frame = frame_stack_k - 1
    else: start_frame = start_time_idx
    if start_frame < frame_stack_k - 1: start_frame = frame_stack_k - 1
    if start_frame >= total_steps:
        logging.error(f"start_time_idx ({start_frame}) is beyond trajectory length ({total_steps}).")
        return np.array([]), np.array([]), {}

    max_possible_steps = total_steps - start_frame - 1
    rollout_steps = min(horizon or max_possible_steps, max_possible_steps)
    
    if rollout_steps <= 0:
        logging.warning(f"Rollout has 0 steps to run.")
        return np.array([]), np.array([]), {}
        
    # Initialize proprioception history (q_pred_history)
    history_start_idx = start_frame - frame_stack_k + 1
    history_end_idx = start_frame + 1
    
    start_state_proprio = state_t[start_frame, PROP_START_IDX:PROP_END_IDX]
    if mimic_rollout_start:
        q_pred_history = start_state_proprio.unsqueeze(0).repeat(frame_stack_k, 1)
        logging.info(f"Rollout starting at frame {start_frame} for {rollout_steps} steps (Mimicking static start).")
    else:
        q_pred_history = state_t[history_start_idx:history_end_idx, PROP_START_IDX:PROP_END_IDX].clone()
        logging.info(f"Rollout starting at frame {start_frame} for {rollout_steps} steps (Using ground-truth history).")
    
    predicted_q_trajectory = []
    is_sequence_model = any(k in model.__class__.__name__.lower() for k in ['lstm', 'gru', 'transformer', 'rnn'])
    arm_action_dim = 6 if control_mode == 'task_space' else 7
    
    for i in range(rollout_steps):
        t = start_frame + i

        # --- 2. BUILD CORE STATE (P_pred for Proprio-Only; T_gt|P_pred|V_gt for Full) ---
        current_history_start_idx = t - frame_stack_k + 1
        current_history_end_idx = t + 1
        
        if proprio_only:
            # Core state is just the predicted proprio history (K, 23)
            core_state_sequence = q_pred_history
        else:
            # Core state is the full state with predicted proprio injected
            tactile_gt_stack = state_t[current_history_start_idx : current_history_end_idx, 0:TACTILE_DIM]
            visual_gt_stack = state_t[current_history_start_idx : current_history_end_idx, VISUAL_START_IDX:state_dim]
            
            # RECONSTRUCT STATE AS [T_gt | P_pred | V_gt] 
            core_state_sequence = torch.cat([
                tactile_gt_stack,
                q_pred_history, 
                visual_gt_stack 
            ], dim=1) 
        
        # 3. Add goal if required
        if use_goal:
            goal_window = goal_t.unsqueeze(0).repeat(frame_stack_k, 1)
            # Concatenate core state (filtered or full) with the goal window
            full_state_sequence = torch.cat([core_state_sequence, goal_window], dim=1)
        else:
            full_state_sequence = core_state_sequence

        # 4. Apply normalization (X_mean/X_std)
        if is_sequence_model:
            x_tensor = ((full_state_sequence - X_mean) / (X_std + 1e-8)).unsqueeze(0)
        else:
            # For MLP, flatten the k frames
            flat_state = full_state_sequence.flatten()
            norm_flat = (flat_state - X_mean.repeat(frame_stack_k)) / (X_std.repeat(frame_stack_k) + 1e-8)
            x_tensor = norm_flat.unsqueeze(0)

        # 5. Model forward and De-normalize (Y_mean/Y_std) ---
        with torch.no_grad():
            if use_average_policy:
                y_pred_norm = torch.zeros_like(y_std) 
            else:
                # Execution happens here: x_tensor must match model's input dim
                y_pred_norm = model(x_tensor).squeeze(0)

            delta_action_pred = (y_pred_norm * y_std) + y_mean
            
        # 6. Propagate next proprio ---
        last_q_full_proprio = q_pred_history[-1] 

        # The model output is y_pred (either Delta Action or Absolute Pose)
        y_pred = delta_action_pred 
        
        # --- Separate Arm and Hand DOFs in the prediction output ---
        arm_pred = y_pred[:arm_action_dim]
        if is_arm_only:
            hand_pred = torch.zeros(HAND_JOINT_DIM, device=device)
        else:
            hand_pred = y_pred[arm_action_dim:]
            
        # --- Get the current state for propagation (needed for Delta) ---
        last_q_arm = last_q_full_proprio[:ARM_PROP_DIM]
        last_q_hand = last_q_full_proprio[ARM_PROP_DIM:]

        # -----------------------------------------------------------------
        # --- PROPAGATION LOGIC: DELTA vs. ABSOLUTE ---
        # -----------------------------------------------------------------
        if action_space == 'absolute':
            # Model output IS the next state: q_t+1_pred = y_pred
            
            # 1. Hand joints are direct absolute state prediction
            next_q_hand_raw = hand_pred
            
            if control_mode == 'joint_space':
                # Joint Space Absolute: Output is 7D Joint Angles (q_t+1). History stores 7D Joints.
                next_q_arm_raw = arm_pred
            elif control_mode == 'task_space':
                # Task Space Absolute:
                # Arm prediction (arm_pred) is 6D: [x, y, z, r1, r2, r3]
                
                # --- NEW ABSOLUTE CARTESIAN PROPAGATION ---
                if solver is None:
                    raise ValueError("KinematicsSolver 'solver' is None, but control_mode is 'task_space'.")

                # a. Convert last 7D proprio state (Pos+Quat) to T_current (SE3)
                T_current = pin.XYZQUATToSE3(to_np(last_q_arm))
                
                # b. Convert 6D predicted pose (Pos+RotVec) to T_target (SE3)
                p_target = to_np(arm_pred[:3])
                r_target_vec = to_np(arm_pred[3:])

                # --- SIGN CORRECTION / CLOSEST ROTATION CHECK ---
                from scipy.spatial.transform import Rotation as R
                R_target = R.from_rotvec(r_target_vec)
                
                # Convert the current state's orientation (from T_current) to RotVec to compare
                T_current_np = pin.SE3ToXYZQUAT(T_current)
                q_current = T_current_np[3:]
                R_current = R.from_quat(q_current)
                r_current_vec = R_current.as_rotvec()

                # Determine if the predicted rotation R_target or its inverse (-R_target) is closer to R_current
                # This is the same logic used in evaluation for absolute targets.
                r_target_vec_flip = R.from_quat(-R_target.as_quat()).as_rotvec() # R(-r)
                
                dist_direct = np.linalg.norm(r_target_vec - r_current_vec)
                dist_flip = np.linalg.norm(r_target_vec_flip - r_current_vec)
                
                if dist_flip < dist_direct:
                    r_target_vec = r_target_vec_flip
                    R_target = R.from_rotvec(r_target_vec)

                # Rebuild the final T_target (SE3) with the corrected rotation
                quat_target_corrected = R_target.as_quat()
                q_target_7d_corrected = np.concatenate([p_target, quat_target_corrected])
                T_target = pin.XYZQUATToSE3(q_target_7d_corrected)

                # c. Interpolate between T_current and T_target to get T_next (Optional but safer)
                # T_next = T_current * exp(log(T_current^-1 * T_target)) * alpha
                # For simplicity and direct action: T_next = T_target (since alpha=1 is implicit)
                T_next = T_target
                
                # d. Convert T_next (SE3) back to 7D proprioception (Pos+Quat)
                next_pose_np = pin.SE3ToXYZQUAT(T_next)
                
                next_q_arm_raw = torch.as_tensor(next_pose_np, dtype=torch.float32, device=device)
            else:
                raise ValueError(f"Unknown control_mode: {control_mode}")
            
        elif action_space == 'delta':
            # Model output IS the delta: q_t+1 = q_t + Delta_pred
            next_q_hand_raw = last_q_hand + hand_pred

            if control_mode == 'joint_space':
                next_q_arm_raw = last_q_arm + arm_pred
            elif control_mode == 'task_space':
                # The task-space logic uses pinocchio to integrate the delta pose:
                if solver is None:
                    raise ValueError("KinematicsSolver 'solver' is None, but control_mode is 'task_space'.")
                    
                T_current = pin.XYZQUATToSE3(to_np(last_q_arm))
                T_delta = pin.exp(to_np(arm_pred)) # arm_pred is the delta twist
                T_next = T_current * T_delta
                next_pose_np = pin.SE3ToXYZQUAT(T_next)
                
                if next_pose_np[6] < 0: next_pose_np[3:] *= -1
                next_q_arm_raw = torch.as_tensor(next_pose_np, dtype=torch.float32, device=device)
            else:
                 raise ValueError(f"Unknown control_mode: {control_mode}")
        
        else:
            raise ValueError(f"Unknown action_space: {action_space}")
        
        # -----------------------------------------------------------------
        
        # --- Apply Joint Limits (Clamping) ---
        
        if control_mode == 'joint_space':
             # Only clamp arm joints if in joint_space control mode
             next_q_arm = torch.clamp(next_q_arm_raw, joint_min[:num_arm_joints], joint_max[:num_arm_joints])
        else:
             # In task_space, the arm prediction is already the absolute pose [x, y, z, qx, qy, qz, qw]
             # No clamping is typically done here on the pose itself, as inverse kinematics handles joint limits.
             # We just pass the raw prediction.
             next_q_arm = next_q_arm_raw

        next_q_hand = torch.clamp(
            next_q_hand_raw, 
            joint_min[num_arm_joints:], 
            joint_max[num_arm_joints:]
        )
        
        q_pred_next = torch.cat([next_q_arm, next_q_hand])        
        predicted_q_trajectory.append(to_np(q_pred_next))
        q_pred_history = torch.roll(q_pred_history, shifts=-1, dims=0)
        q_pred_history[-1] = q_pred_next

    # --- Metrics Calculation (unchanged) ---
    pred_np = np.array(predicted_q_trajectory)
    
    gt_start_idx = start_frame + 1
    gt_end_idx = start_frame + 1 + rollout_steps
    gt_np = to_np(state_t[gt_start_idx : gt_end_idx, PROP_START_IDX:PROP_END_IDX])

    if len(pred_np) == 0 or len(gt_np) == 0:
        logging.warning("Rollout or GT has length 0. Returning empty metrics.")
        return np.array([]), np.array([]), {'mse': np.nan, 'mae': np.nan, 'r2': np.nan}
    
    if len(pred_np) != len(gt_np):
        min_len = min(len(pred_np), len(gt_np))
        pred_np = pred_np[:min_len]
        gt_np = gt_np[:min_len]

    if is_arm_only:
        gt_for_metrics = gt_np[:, :ARM_PROP_DIM]
        pred_for_metrics = pred_np[:, :ARM_PROP_DIM]
    else:
        gt_for_metrics = gt_np
        pred_for_metrics = pred_np

    metrics = {
        'mse': mean_squared_error(gt_for_metrics, pred_for_metrics),
        'mae': mean_absolute_error(gt_for_metrics, pred_for_metrics),
        'r2': r2_score(gt_for_metrics, pred_for_metrics)
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
    parser.add_argument("--start_step", type=int, default=None)
    parser.add_argument("--mimic_rollout_start", action="store_true")
    parser.add_argument("--dummy", action="store_true")
    parser.add_argument("--proprio_only", action="store_true", help="If set, use only proprioceptive data for evaluation.")
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

        action_space = config.get("action_space", "delta")
        
        # --- Read hyperparameters from the checkpoint ---
        model_hyperparams = checkpoint.get("model_hyperparams", {})
        logging.info(f"Loaded model hyperparameters: {model_hyperparams}")
        logging.info(f"Model was trained in {action_space} {control_mode}, with K={frame_stack_k}, type='{model_type}' and {'WITH' if validation else 'WITHOUT'} validation loss. Evaluating accordingly.")
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

        # --- NEW: Define a flag to trigger the 7D -> 6D conversion ---
        # This flag should only be TRUE if the model was trained with the 6D Cartesian target
        should_convert_7d_to_6d = (action_space == 'absolute' and control_mode == 'task_space')

        # Check if the model's output dimension (joint_dim) is 22 (6+16) or 6 (arm_only)
        # This is a sanity check to ensure the model matches the conversion logic.
        if should_convert_7d_to_6d and not (joint_dim == 6 and is_arm_only) and not (joint_dim == 22 and not is_arm_only):
            logging.warning(
                f"Model output dim ({joint_dim}) does not match expected 6D target dim (6 or 22) "
                f"for absolute task_space mode. Proceeding but check data loading consistency."
            )

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

        # # --- Slice the validation data if it's an arm-only model ---
        # if is_arm_only:
        #     for traj in val_trajectories:
        #         # traj['joints_t'] = traj['joints_t'][:, :num_arm_joints]
        #         if control_mode == "joint_space":
        #             traj['action_t'] = traj['action_t'][:, :num_arm_joints]
        #         # if 'goal_t' in traj:
        #         #     # Reconstruct the correct sliced goal dimension
        #         #     goal_state_dim = traj['tactile_t'].shape[1] + traj['visual_t'].shape[1] + num_arm_joints
        #         #     traj['goal_t'] = traj['goal_t'][:, :goal_state_dim]
        #         else:
        #             traj['action_t'] = traj['action_t'][:, :6]

        output_dir = paths.MODELS_DIR / "debug"
        output_dir.mkdir(parents=True, exist_ok=True)
        logging.info(f"Loaded {len(val_trajectories)} validation trajectories from dataset.")
        logging.info(f"Evaluation outputs will be saved to: {output_dir}")
        
        # --- ONE-STEP MODE ---
        logging.info("Starting one-step evaluation with frame stacking...")
        is_sequence_model = model_type in ["lstm", "gru", "transformer"]
        flatten_data = not is_sequence_model
        
        # --- Define state indices (needed for filtering) ---
        TACTILE_DIM = 24
        ARM_PROP_DIM = 7
        HAND_JOINT_DIM = 16
        FULL_PROP_DIM = ARM_PROP_DIM + HAND_JOINT_DIM # 23
        PROP_START_IDX = TACTILE_DIM 
        PROP_END_IDX = PROP_START_IDX + FULL_PROP_DIM # 24 + 23 = 47

        X_val_list, y_val_list = [], []
        for traj in val_trajectories:
            state_dim_for_goal = traj['state_t'].shape[1]
            if config.get('use_goal', False):
                X_unstacked = np.concatenate([traj['state_t'], traj['goal_t']], axis=1)
                goal_dim = traj['goal_t'].shape[1]
            else:
                X_unstacked = traj['state_t']
                goal_dim = 0

            # --- PROPRIO-ONLY FILTER (for One-Step) ---
            if final_args.proprio_only:
                proprio_indices = np.arange(PROP_START_IDX, PROP_END_IDX) # 24...46
                if config.get('use_goal', False):
                    goal_indices = np.arange(state_dim_for_goal, state_dim_for_goal + goal_dim)
                    final_indices = np.concatenate([proprio_indices, goal_indices])
                else:
                    final_indices = proprio_indices
                X_unstacked = X_unstacked[:, final_indices]

            if action_space == 'absolute':
                # The true target is the proprioception of the NEXT state (Q_t+1)
                
                # 1. Get the full state sequence Q_1 to Q_T (excluding Q_0)
                q_next_full = traj['state_t'][1:] 
                
                # 2. Slice the proprioception features (7D Arm + 16D Hand)
                y_unstacked_full_7d = q_next_full[:, PROP_START_IDX : PROP_END_IDX]
                
                y_unstacked = []
                if should_convert_7d_to_6d:
                    # --- CONVERSION LOGIC FOR ABSOLUTE TASK SPACE (7D -> 6D Target) ---
                    from scipy.spatial.transform import Rotation as R
                    
                    for i in range(y_unstacked_full_7d.shape[0]):
                        q_next_7d = y_unstacked_full_7d[i, :7]
                        q_next_hand_16d = y_unstacked_full_7d[i, 7:]
                        
                        # Convert 4D Quaternion to 3D Rotation Vector
                        p_next = q_next_7d[:3]
                        quat_next = q_next_7d[3:] # [qx, qy, qz, qw]
                        # Check for zero quaternion norm to prevent error
                        if np.linalg.norm(quat_next) < 1e-6:
                            # If the quaternion is zero, treat the rotation as zero
                            r_next_vec = np.zeros(3, dtype=np.float32)
                        else:
                            r_next_vec = R.from_quat(quat_next).as_rotvec() # 3D Rot Vector

                        # Recombine to the 6D arm target
                        action_gt_arm_6d = np.concatenate([p_next, r_next_vec], axis=0)
                        
                        # Recombine with hand (if not arm_only)
                        if is_arm_only:
                            y_unstacked.append(action_gt_arm_6d) # 6D
                        else:
                            action_gt_full_22d = np.concatenate([action_gt_arm_6d, q_next_hand_16d], axis=0)
                            y_unstacked.append(action_gt_full_22d) # 22D

                    y_unstacked = np.stack(y_unstacked, axis=0)
                    
                else:
                    # --- ORIGINAL LOGIC (Delta or Absolute Joint Space) ---
                    y_unstacked = y_unstacked_full_7d

                    # 3. Apply ARM-ONLY slicing if the model was trained arm-only
                    if is_arm_only:
                        # num_arm_joints is 7 for joint_space, 6 for task_space (delta)
                        y_unstacked = y_unstacked[:, :num_arm_joints] # Use the sliced dimension from the model
            
            elif action_space == 'delta':
                # The true target is the DELTA action from the dataset (Q_t+1 - Q_t)
                y_unstacked = traj['action_t']
                # Arm-only slicing was already applied to traj['action_t'] earlier in the script.
                
            else:
                logging.error(f"Unknown action_space '{action_space}'")
                continue

            if len(X_unstacked) < frame_stack_k: continue

            # len(y_unstacked) is T-1. Indices run from 0 to T-2.
            # We use i to index y_unstacked and the CURRENT state stack.
            for i in range(frame_stack_k - 1, len(y_unstacked)): 
                
                # The state sequence ends at index i.
                state_sequence = X_unstacked[i - frame_stack_k + 1 : i + 1]
                X_val_list.append(state_sequence.flatten() if flatten_data else state_sequence)
                
                # The action/target is at index i, which is correct for the new loop bounds.
                y_val_list.append(y_unstacked[i])

        # Convert to standard numpy arrays first
        X_val_np = np.array(X_val_list)
        y_val_np = np.array(y_val_list)

        # Move tensors to the device *only when needed* (in the loop or DataLoader)
        # For now, keep them on the CPU to create the DataLoader
        X_val_cpu = torch.tensor(X_val_np, dtype=torch.float32) 
        y_val_cpu = torch.tensor(y_val_np, dtype=torch.float32)

        # Create a TensorDataset and DataLoader
        # Batch size is the key to GPU memory efficiency. 
        # Start with a safe number (e.g., 512, 1024, or 2048) and increase if performance allows.
        BATCH_SIZE = 1024 # Adjust this value!

        val_dataset = TensorDataset(X_val_cpu, y_val_cpu)
        val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False)

        logging.info(f"Created DataLoader with {len(val_dataset)} samples and batch size {BATCH_SIZE}.")

        # Initialize lists to collect results on the CPU
        action_pred_list, y_val_true_list = [], []      

        X_mean, X_std, y_mean, y_std = norm_stats

        with torch.no_grad():
            for X_batch_raw, y_batch_true in val_loader:
                # Move the batch data to the GPU (CUDA)
                X_batch_raw = X_batch_raw.to(device)
                y_batch_true = y_batch_true.to(device)

                # Normalization logic
                if flatten_data: # MLP
                    # Ensure the normalisation tensor matches the stacked dimension
                    X_val_norm = (X_batch_raw - X_mean.repeat(frame_stack_k)) / X_std.repeat(frame_stack_k)

                else: # Sequence models (e.g., LSTM, GRU, Transformer)
                    # X_batch_raw shape: (Batch, SeqLen, Features)
                    # X_mean/std shape: (Features,)
                    
                    # Reshape mean/std to (1, 1, Features) for correct broadcasting across Batch and SeqLen dimensions.
                    X_mean_reshaped = X_mean.view(1, 1, -1)
                    X_std_reshaped = X_std.view(1, 1, -1)
                    
                    X_val_norm = (X_batch_raw - X_mean_reshaped) / X_std_reshaped
                # Model prediction
                if final_args.dummy:
                    pred_norm = torch.zeros(X_val_norm.shape[0], joint_dim, device=device)
                else:
                    pred_norm = model(X_val_norm)

                # Denormalization
                action_pred = (pred_norm * y_std) + y_mean
                
                # Collect results, moving them back to the CPU
                action_pred_list.append(action_pred.cpu())
                y_val_true_list.append(y_batch_true.cpu())

        # Concatenate all batches into final CPU numpy arrays
        action_pred_np = to_np(torch.cat(action_pred_list))
        action_true_np = to_np(torch.cat(y_val_true_list))    
            
        # --- NEW: Orientation Sign Correction for Task Space Absolute Mode ---
        if should_convert_7d_to_6d:
            # Determine the number of dimensions for the arm part (6)
            arm_action_dim = 6 
            
            # 1. Separate the arm components
            action_true_arm = action_true_np[:, :arm_action_dim]
            action_pred_arm = action_pred_np[:, :arm_action_dim]
            
            # 2. Iterate over the arm predictions to apply the sign flip correction
            # We only apply this to the orientation components (indices 3, 4, 5)
            
            # Find the indices for the orientation components
            # The start index for orientation is 3.
            ori_start_idx = 3 

            # Calculate the squared error for the direct prediction (e_direct)
            # and the squared error for the flipped prediction (e_flipped)
            # We only care about the orientation part for the flip decision.

            # Calculate squared error for orientation components (3, 4, 5)
            e_direct_sq = np.sum((action_true_arm[:, ori_start_idx:] - action_pred_arm[:, ori_start_idx:])**2, axis=1)
            e_flipped_sq = np.sum((action_true_arm[:, ori_start_idx:] - (-action_pred_arm[:, ori_start_idx:]))**2, axis=1)

            # 3. Apply the correction where the flipped prediction is better
            flip_indices = e_flipped_sq < e_direct_sq
            
            # Apply the flip to the prediction (only for orientation components 3, 4, 5)
            action_pred_arm[flip_indices, ori_start_idx:] *= -1
            
            # 4. Recombine the corrected arm prediction with the hand prediction (if any)
            if not is_arm_only:
                # Hand prediction is unchanged (indices 6 to end)
                action_pred_np_corrected = np.concatenate([action_pred_arm, action_pred_np[:, arm_action_dim:]], axis=1)
            else:
                action_pred_np_corrected = action_pred_arm

            # Use the corrected array for metric calculation
            metrics = {'mse': mean_squared_error(action_true_np, action_pred_np_corrected), 
                        'mae': mean_absolute_error(action_true_np, action_pred_np_corrected), 
                        'r2': r2_score(action_true_np, action_pred_np_corrected)}

        else:
            # Use the original, uncorrected arrays for all other modes (delta, joint space)
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
            # --- FIX: Update Label for Absolute Pose ---
            if action_space == 'absolute':
                arm_label = "q (rad)" # Absolute joint angle
            else:
                arm_label = "Δq (rad)"
                
            arm_names = [f"Joint {i+1}" for i in range(arm_dim)]
        else: # task_space
            arm_dim = num_arm_task_dof
            # --- FIX: Update Label for Absolute Pose ---
            if action_space == 'absolute':
                # Assuming your action output is position (3) + orientation (3/4), 
                # but your plotting only shows 6D here (Δx/rad). Be explicit.
                arm_label = "x/r (m / rad)" # Absolute position/orientation
            else:
                arm_label = "Δx/Δr (m / rad)"
                
            arm_names = ["x", "y", "z", "roll", "pitch", "yaw"] # Update names to reflect absolute

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
            hand_to_plot = list(range(16))  # Plot all 16 hand joints
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

        # Inside the Scatter plots loop:
        # Check which array to use for plotting
        if should_convert_7d_to_6d:
            pred_data = action_pred_np_corrected
        else:
            pred_data = action_pred_np

        # --- Scatter plots per DOF ---
        for i, (name, idx) in enumerate(zip(plot_names, plot_indices)):
            ax = axes[i]
            if idx >= action_true_np.shape[1]: continue
            
            # Determine the appropriate unit label
            if idx < arm_dim:
                # Arm DOF: use the control_mode specific label
                unit_label = arm_label
            else:
                # Hand DOF: always joint delta
                unit_label = "Δq (rad)" # Use the joint delta label

            ax.scatter(action_true_np[:, idx], pred_data[:, idx], alpha=0.15, s=10) # CHANGED: from action_pred_np to pred_data
            mn, mx = np.min(action_true_np[:, idx]), np.max(action_true_np[:, idx])
            ax.plot([mn, mx], [mn, mx], "r--", linewidth=1, label="Perfect Prediction")
            ax.set_title(name)
            ax.set_xlabel(f"Ground Truth ({unit_label})")
            ax.set_ylabel(f"Predicted ({unit_label})")
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
                    use_goal=config.get("use_goal", False),
                    solver=solver,
                    action_space=action_space,
                    mimic_rollout_start=final_args.mimic_rollout_start,
                    start_time_idx=final_args.start_step,
                    use_average_policy=final_args.dummy,
                    proprio_only=final_args.proprio_only
                )
                all_metrics.append(metrics)
                all_preds.append(pred_np)
                all_gts.append(gt_np)
                logging.info(f"  Trajectory {i+1}/{len(val_trajectories)} | MAE = {metrics['mae']} | R² = {metrics['r2']:.4f}")

                if i >= 10:       # only plot first 10
                    continue

                # --- DATA PREPARATION FOR PLOTTING ---
                # By default, use the raw numpy arrays
                gt_to_plot = gt_np
                pred_to_plot = pred_np
                
                # If in task space, convert quaternions to 6D Pos+RotVec for a smooth plot
                if control_mode == 'task_space':
                    # Prepare empty arrays for position + rotation vector
                    gt_plot_data = np.zeros((gt_np.shape[0], 6))
                    pred_plot_data = np.zeros((pred_np.shape[0], 6))

                    # Copy over the position data (x, y, z)
                    gt_plot_data[:, :3] = gt_np[:, :3]
                    pred_plot_data[:, :3] = pred_np[:, :3]
                    
                    # --- STEP 1: Convert Quaternions to Rotation Vector ---
                    from scipy.spatial.transform import Rotation as R

                    # 1a. GT Conversion (Canonicalize Quat before RotVec)
                    gt_quat = gt_np[:, 3:7].copy()
                    gt_quat[gt_quat[:, 3] < 0] *= -1 # Canonicalize w-component
                    gt_rotvec = np.unwrap(R.from_quat(gt_quat).as_rotvec())
                    
                    # 1b. Prediction Conversion (Canonicalize Quat before RotVec)
                    pred_quat = pred_np[:, 3:7].copy()
                    # We must apply the sign correction here to align prediction with GT
                    
                    # The pred_quat from the rollout history already carries the Quat sign from pinocchio/propagation,
                    # but we must ensure it's the *canonical* representation for plotting the RotVec.
                    pred_quat[pred_quat[:, 3] < 0] *= -1 
                    pred_rotvec = np.unwrap(R.from_quat(pred_quat).as_rotvec())

                    # --- STEP 2: Place the Rotation Vectors into the plot data ---
                    gt_plot_data[:, 3:] = gt_rotvec
                    pred_plot_data[:, 3:] = pred_rotvec
                    
                    # Update the variables used for plotting
                    gt_to_plot = gt_plot_data
                    pred_to_plot = pred_plot_data
                
                # --- START ARM PLOTTING LOGIC (Unified File) ---
                
                if control_mode == "joint_space":
                    # ---------- Arm (Joint Space) - Standard 7-DOF Plot ----------
                    fig, ax = plt.subplots(figsize=(12, 6))
                    for j in range(arm_dim): # arm_dim is 7
                        c = f"C{j%10}"
                        # UPDATED: Use the processed data for plotting
                        ax.plot(gt_to_plot[:, j], c=c, ls='--', label=f"GT {arm_names[j]}")
                        ax.plot(pred_to_plot[:, j], c=c, label=f"Pred {arm_names[j]}")
                    
                    fig.suptitle(f"Rollout vs GT – Joint Angles (Traj {i+1})", fontsize=14)
                    ax.set(xlabel="Timestep", ylabel=arm_label)
                    ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1))
                    ax.grid(ls='--')
                    
                    plt.tight_layout(rect=[0, 0, 1, 0.95]) # Adjust for suptitle
                    plt.savefig(rollout_plot_dir / f"rollout_traj_{i+1}_arm.png")
                    plt.close(fig)

                else: # control_mode == "task_space"
                    # ---------- Arm (Task Space) - Split Position and Orientation ----------
                    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
                    
                    # --- Subplot 1: Position (X, Y, Z) ---
                    pos_names = arm_names[:3] # ["x", "y", "z"]
                    for j in range(3):
                        c = f"C{j%10}"
                        axes[0].plot(gt_to_plot[:, j], c=c, ls='--', label=f"GT {pos_names[j]}")
                        axes[0].plot(pred_to_plot[:, j], c=c, label=f"Pred {pos_names[j]}")
                    
                    axes[0].set_title("EE Position (XYZ)")
                    axes[0].set_ylabel("Position (m)")
                    axes[0].legend(loc="upper right", bbox_to_anchor=(1.1, 1))
                    axes[0].grid(ls='--')

                    # --- Subplot 2: Orientation (Rotation Vector Components) ---
                    # The names must be updated to reflect RotVec components
                    ori_names = ["r_x", "r_y", "r_z"] # Changed from "roll", "pitch", "yaw"
                    for j in range(3):
                        c = f"C{j%10}"
                        # Plot columns 3, 4, 5 from the 6D data (Pos+RotVec)
                        axes[1].plot(gt_to_plot[:, j+3], c=c, ls='--', label=f"GT {ori_names[j]}")
                        axes[1].plot(pred_to_plot[:, j+3], c=c, label=f"Pred {ori_names[j]}")
                    
                    axes[1].set_title("EE Orientation (Rotation Vector)")
                    axes[1].set_xlabel("Timestep")
                    axes[1].set_ylabel("Angle * Axis (rad)") # Updated Y-label
                    axes[1].legend(loc="upper right", bbox_to_anchor=(1.1, 1))
                    axes[1].grid(ls='--')
                    
                    fig.suptitle(f"Rollout vs GT – Cartesian Pose (Traj {i+1})", fontsize=14)                    
                    plt.tight_layout(rect=[0, 0, 1, 0.95]) # Adjust for suptitle
                    
                    # --- SAVING TO THE UNIFIED FILENAME ---
                    plt.savefig(rollout_plot_dir / f"rollout_traj_{i+1}_arm.png")
                    plt.close(fig)
                
                # ---------- Hand ----------
                if not is_arm_only and gt_np.shape[1] > ARM_PROP_DIM: 
                    fig, ax = plt.subplots(figsize=(12,6))
                    hand_start = ARM_PROP_DIM # <-- FIX: This must be 7 (0-indexed)
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

                num_to_plot = min(10, len(valid_pairs))
                ARM_PROP_DIM = 7  # Fixed dimension of arm proprio state (e.g., XYZ-Quat or 7 Joint Angles)
                num_hand_joints = 16
                total_dofs = ARM_PROP_DIM if is_arm_only else (ARM_PROP_DIM + num_hand_joints) # total_dofs is 7 or 23

                fig, axes = plt.subplots(num_to_plot, 1, figsize=(10, 4*num_to_plot), sharex=True)
                if num_to_plot == 1:
                    axes = [axes]

                for idx in range(num_to_plot):
                    pred_np, gt_np = valid_pairs[idx]

                    # --- Always slice the relevant DOFs ---
                    pred_np = pred_np[:, :total_dofs]
                    gt_np   = gt_np[:, :total_dofs]

                    if is_arm_only:
                        # --- Arm-only drift (Joint or Cartesian combined) ---
                        drift_arm = np.sqrt(np.mean((pred_np - gt_np) ** 2, axis=1))
                        axes[idx].plot(drift_arm, label="Arm Drift (All DOFs)", color="C0")
                    
                    elif control_mode == "joint_space":
                        # --- Joint Space: Single Arm RMSE + Hand RMSE ---
                        
                        drift_arm  = np.sqrt(np.mean(
                            (pred_np[:, :ARM_PROP_DIM] - gt_np[:, :ARM_PROP_DIM]) ** 2, axis=1
                        ))
                        drift_hand = np.sqrt(np.mean(
                            (pred_np[:, ARM_PROP_DIM:total_dofs] - gt_np[:, ARM_PROP_DIM:total_dofs]) ** 2, axis=1
                        ))
                        axes[idx].plot(drift_arm,  label="Arm Drift (Joint RMSE)",  color="C0")
                        axes[idx].plot(drift_hand, label="Hand Drift (Joint RMSE)", color="C1")
                    
                    else: # control_mode == "task_space"
                        # --- Cartesian Space: Position RMSE + Orientation RMSE + Hand RMSE ---
                        
                        # Arm Position (First 3 DOFs: X, Y, Z) - UNCHANGED
                        drift_arm_pos = np.sqrt(np.mean(
                            (pred_np[:, :3] - gt_np[:, :3]) ** 2, axis=1
                        ))
                        
                        # Arm Orientation (DOFs 3 to 7: Quat) - FIX APPLIED HERE
                        pred_ori = pred_np[:, 3:ARM_PROP_DIM] # (N, 4)
                        gt_ori = gt_np[:, 3:ARM_PROP_DIM]     # (N, 4)
                        
                        # 1. Calculate the standard squared error (q_pred - q_gt)^2
                        sq_err_std = (pred_ori - gt_ori) ** 2
                        
                        # 2. Calculate the flipped squared error (q_pred - (-q_gt))^2 = (q_pred + q_gt)^2
                        sq_err_flip = (pred_ori + gt_ori) ** 2
                        
                        # 3. Use the MINIMUM of the two squared errors for each timestep and each DOF
                        min_sq_err = np.minimum(sq_err_std, sq_err_flip)
                        
                        # 4. Calculate RMSE using the minimum error
                        drift_arm_ori = np.sqrt(np.mean(min_sq_err, axis=1))

                        # Hand (DOFs 7 onwards) - UNCHANGED
                        drift_hand = np.sqrt(np.mean(
                            (pred_np[:, ARM_PROP_DIM:total_dofs] - gt_np[:, ARM_PROP_DIM:total_dofs]) ** 2, axis=1
                        ))
                        
                        axes[idx].plot(drift_arm_pos, label="Arm Position Drift (RMSE)", color="C0")
                        axes[idx].plot(drift_arm_ori, label="Arm Orientation Drift (RMSE on Quat)", color="C2")
                        axes[idx].plot(drift_hand, label="Hand Drift (RMSE)", color="C1")
                        
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

                if control_mode == "task_space":
                    # Initialize error array
                    squared_error = (all_pred - all_gt) ** 2

                    # Target indices for Quaternion DOFs (3, 4, 5, 6)
                    ORI_START_IDX = 3
                    ORI_END_IDX = ARM_PROP_DIM # Which is 7

                    # Extract orientation parts
                    pred_ori = all_pred[:, ORI_START_IDX : ORI_END_IDX]
                    gt_ori = all_gt[:, ORI_START_IDX : ORI_END_IDX]

                    # 1. Standard squared error
                    sq_err_std = (pred_ori - gt_ori) ** 2
                    # 2. Flipped squared error
                    sq_err_flip = (pred_ori + gt_ori) ** 2

                    # 3. Use the MINIMUM error for orientation
                    min_sq_err = np.minimum(sq_err_std, sq_err_flip)

                    # 4. Inject the corrected minimum squared error back into the full array
                    squared_error[:, ORI_START_IDX : ORI_END_IDX] = min_sq_err

                    # Calculate overall RMSE per DOF using the corrected squared error
                    rmse_per_dof = np.sqrt(np.mean(squared_error, axis=0))

                else: # Joint space or arm_only (standard RMSE is fine)
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