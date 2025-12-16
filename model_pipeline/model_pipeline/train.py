from model_pipeline import paths
from pathlib import Path
import argparse
import logging
import inquirer
import pickle
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
import random
import matplotlib.pyplot as plt
import yaml
import math
import os

from model_pipeline.utils import find_config_files
from model_pipeline import paths
from model_pipeline.utils import find_pkl_files

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("Warning: Pinocchio not installed. Task-space drift regularization will not work.")

num_cpus = os.cpu_count() # Get the number of available cores
TARGET_WORKERS = min(4, num_cpus // 2) 

# Fixed dimension constants
TACTILE_DIM = 24
ARM_PROP_DIM = 7
HAND_PROP_DIM = 16
FULL_PROP_DIM = ARM_PROP_DIM + HAND_PROP_DIM # 23
ARM_PROP_SLICE_IN_PROP_VECTOR = slice(0, ARM_PROP_DIM) # 0:7 for arm features within a 23D proprio vector


# --- Logger Setup ---
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

# ===================================================================
# === MODEL DEFINITIONS ===
# ===================================================================
class MLPPolicy(nn.Module):
    """A dynamically generated MLP with a configurable number of layers and width."""
    def __init__(self, input_dim, output_dim, width=512, num_layers=2, dropout_p=0.2):
        super().__init__()
        
        layers = []
        # --- Input Layer ---
        layers.append(nn.Linear(input_dim, width))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(p=dropout_p))
        
        # --- Hidden Layers ---
        # This loop creates the tapering structure
        current_width = width
        for _ in range(num_layers - 1):
            next_width = current_width // 2
            layers.append(nn.Linear(current_width, next_width))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(p=dropout_p))
            current_width = next_width
            
        # --- Output Layer ---
        layers.append(nn.Linear(current_width, output_dim))
        
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)

class LSTMPolicy(nn.Module):
    """A more robust LSTM Policy for sequence data."""
    def __init__(self, input_dim, output_dim, hidden_dim=256, num_layers=2, dropout_p=0.2):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True, dropout=0.2 if num_layers > 1 else 0)
        self.dropout1 = nn.Dropout(p=dropout_p)
        self.fc1 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.relu = nn.ReLU()
        self.dropout2 = nn.Dropout(p=dropout_p)
        self.fc2 = nn.Linear(hidden_dim // 2, output_dim)
    def forward(self, x):
        lstm_out, _ = self.lstm(x)
        last_timestep_out = lstm_out[:, -1, :]
        x = self.dropout1(last_timestep_out)
        x = self.relu(self.fc1(x))
        x = self.dropout2(x)
        return self.fc2(x)
    
class GRUPolicy(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dim=256, num_layers=2, dropout_p=0.2):
        super().__init__()
        self.gru = nn.GRU(input_dim, hidden_dim, num_layers, batch_first=True, dropout=dropout_p if num_layers > 1 else 0)
        self.fc1 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden_dim // 2, output_dim)
        self.dropout_out = nn.Dropout(p=dropout_p) 

    def forward(self, x):
        gru_out, _ = self.gru(x)
        last_timestep_out = gru_out[:, -1, :]
        x = self.relu(self.fc1(last_timestep_out))
        x = self.dropout_out(x)
        return self.fc2(x)
    
class PositionalEncoding(nn.Module):
    """
    A standard sinusoidal positional encoding module.
    This is an alternative to learned embeddings.
    """
    def __init__(self, d_model, max_len=50):
        super().__init__()
        position = torch.arange(max_len).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2) * (-math.log(10000.0) / d_model))
        pe = torch.zeros(1, max_len, d_model)
        pe[0, :, 0::2] = torch.sin(position * div_term)
        pe[0, :, 1::2] = torch.cos(position * div_term)
        self.register_buffer('pe', pe)

    def forward(self, x):
        """
        x: Shape (Batch, SequenceLength, d_model)
        """
        # x.size(1) is the sequence length
        return x + self.pe[:, :x.size(1), :]

class TransformerPolicy(nn.Module):
    """
    A Transformer Policy.
    
    This version includes:
    1. An input projection layer (Linear) to map input_dim -> hidden_dim (d_model).
    2. Learned positional embeddings.
    3. A final output layer that maps hidden_dim -> output_dim.
    """
    def __init__(self, input_dim, output_dim, num_heads=4, hidden_dim=256, num_layers=2, max_seq_len=50):
        super().__init__()
        
        # --- 1. Fix: Project input_dim to hidden_dim (d_model) ---
        # The Transformer's internal dimension (d_model) should be hidden_dim.
        self.input_projection = nn.Linear(input_dim, hidden_dim)
        
        # --- 2. Fix: Add Positional Embeddings ---
        # We need to tell the Transformer the *order* of the states.
        # Using learned embeddings is common. max_seq_len should be >= your frame_stack size.
        self.positional_embeddings = nn.Parameter(torch.zeros(1, max_seq_len, hidden_dim))
        
        # Alternative: Use fixed sinusoidal embeddings
        # self.positional_encoding = PositionalEncoding(hidden_dim, max_seq_len)

        if hidden_dim % num_heads != 0:
            raise ValueError(f"hidden_dim ({hidden_dim}) must be divisible by num_heads ({num_heads}).")

        encoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,  # <-- Use hidden_dim as d_model
            nhead=num_heads, 
            dim_feedforward=hidden_dim * 4, # Common practice
            batch_first=True, 
            dropout=0.1
        )
        self.transformer_encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        
        # --- 3. Fix: Final layer maps from hidden_dim to output_dim ---
        self.output_head = nn.Linear(hidden_dim, output_dim)   
        self.max_seq_len = max_seq_len

    def forward(self, x):
        # x is (Batch, SequenceLength, input_dim)
        if x.size(1) > self.max_seq_len:
             raise ValueError(f"Input sequence length ({x.size(1)}) exceeds model's max_seq_len ({self.max_seq_len})")

        # 1. Project to hidden_dim
        x_proj = self.input_projection(x)  # (Batch, SeqLen, hidden_dim)
        
        # 2. Add positional embeddings
        seq_len = x.size(1)
        x_with_pos = x_proj + self.positional_embeddings[:, :seq_len, :]
        # Alternative: Use fixed sinusoidal embeddings
        # x_with_pos = self.positional_encoding(x_proj)

        # 3. Pass through Transformer
        transformer_out = self.transformer_encoder(x_with_pos) # (Batch, SeqLen, hidden_dim)
        
        # 4. Get the output for the *last* token
        last_timestep_out = transformer_out[:, -1, :] # (Batch, hidden_dim)
        
        # 5. Map to action
        return self.output_head(last_timestep_out) # (Batch, output_dim)

# ===================================================================
# === DATASET CLASS ===
# ===================================================================
class TrajectoryFrameStackDataset(Dataset):
    def __init__(self, trajectories, frame_stack_k, config, arm_only,
                 norm_stats=None, flatten=True, is_train=False,
                 obs_noise_std=0.0, drift_noise_std=0.0):
        
        self.trajectories = trajectories
        self.k = frame_stack_k
        self.flatten = flatten
        self.is_train = is_train
        # obs_noise: small noise applied to ALL features (robustness)
        # drift_noise: large noise applied to PROPRIO (recovery)
        self.obs_noise_std = obs_noise_std 
        self.drift_noise_std = drift_noise_std

        # --- Config options ---
        self.arm_only = arm_only
        self.control_mode = config.get('control_mode', 'joint_space')
        self.use_goal = config.get('use_goal', False)
        self.num_arm_actions = 6 if self.control_mode == 'task_space' else 7

        # --- Constants for Unified Drift Logic ---
        self.DRIFT_PROB = config.get('drift_prob', 0.2) # 20% of samples get large drift
        self.DRIFT_CLIP_TAU = config.get('drift_clip_tau', 4) # Limit corrective action to 4 standard deviations
        self.correction_alpha = config.get('correction_alpha', 0.1) # Scaling factor for correction action

        # --- Action Space ---
        self.action_space = config.get('action_space', 'delta') # 'delta' or 'absolute'

        # --- Pre-calculate Proprioception Slices ---
        TACTILE_DIM = 24
        # In state_t, proprio starts after tactile
        self.proprio_start_idx = TACTILE_DIM 
        
        # Determine how many proprio dimensions we are "drifting"
        # If arm_only, we drift the 7 arm joints/poses. 
        # If full body, we drift everything (23).
        # NOTE: This logic assumes the proprio block is [Arm(7) | Hand(16)]
        drift_dim = self.num_arm_actions if self.arm_only else 23 
        self.proprio_end_idx = self.proprio_start_idx + drift_dim


        # --- Build Indices ---
        self.indices = []
        for traj_idx, traj in enumerate(self.trajectories):
            num_samples = traj['state_t'].shape[0]
            if num_samples >= self.k + 1: 
                for frame_idx in range(self.k - 1, num_samples - 1):
                    self.indices.append((traj_idx, frame_idx))
        
        # --- Normalization Stats ---
        if norm_stats:
            self.X_mean, self.X_std, self.y_mean, self.y_std = norm_stats
        else:
            # Calculate stats from training data
            if self.use_goal:
                X_unstacked = np.concatenate([
                    np.concatenate([t['state_t'], t['goal_t']], axis=1)
                    for t in trajectories
                ], axis=0)
            else:
                X_unstacked = np.concatenate([t['state_t'] for t in trajectories], axis=0)

            # --- Calculate stats for the Action Output (y) ---
            
            # 1. Determine the raw unstacked action/pose data based on self.action_space
            if self.action_space == 'delta':
                # Use the pre-calculated delta actions from the pkl file
                y_unstacked_full = np.concatenate([t['action_t'] for t in trajectories], axis=0)
                
            elif self.action_space == 'absolute':
                # Calculate the absolute target pose (Q_t+1) on the fly
                y_data_list = []
                
                # --- APPLY THE 7D -> 6D CONVERSION LOGIC HERE FOR TASK_SPACE ---
                if self.control_mode == 'task_space':
                    # Import necessary module for conversion
                    from scipy.spatial.transform import Rotation as R
                
                for traj in trajectories:
                    # Q_t+1 proprio features (7D Pos+Quat + 16D Hand Joints)
                    q_next_proprio_full = traj['state_t'][1:, 
                                                          self.proprio_start_idx : self.proprio_end_idx]
                    
                    if self.control_mode == 'joint_space':
                        # Joint Space Absolute (7D Arm) - No conversion needed
                        y_data_list.append(q_next_proprio_full)
                        
                    elif self.control_mode == 'task_space':
                        # Task Space Absolute (6D Arm) - Requires conversion
                        y_traj_list = []
                        for i in range(q_next_proprio_full.shape[0]):
                            q_next_7d = q_next_proprio_full[i, :7]
                            q_next_hand_16d = q_next_proprio_full[i, 7:]
                            
                            # Convert 7D arm to 6D (Pos + RotVec)
                            p_next = q_next_7d[:3]
                            quat_next = q_next_7d[3:] # [qx, qy, qz, qw]
                            r_next_vec = R.from_quat(quat_next).as_rotvec() # 3D Rot Vector

                            action_gt_arm_6d = np.concatenate([p_next, r_next_vec], axis=0)
                            
                            # Recombine with hand
                            action_gt_full = np.concatenate([action_gt_arm_6d, q_next_hand_16d], axis=0)
                            y_traj_list.append(action_gt_full)
                            
                        y_data_list.append(np.stack(y_traj_list, axis=0))

                y_unstacked_full = np.concatenate(y_data_list, axis=0)
            
            else:
                raise ValueError(f"Unknown action_space: {self.action_space}")
                
            # 2. Final slicing based on self.arm_only
            # This is now correct: for task_space, y_unstacked_full is 22D, and self.num_arm_actions=6
            y_unstacked = (y_unstacked_full[:, :self.num_arm_actions]
                           if self.arm_only else y_unstacked_full)

            self.X_mean = X_unstacked.mean(axis=0)
            self.X_std  = X_unstacked.std(axis=0)
            self.y_mean = y_unstacked.mean(axis=0)
            self.y_std  = y_unstacked.std(axis=0)
            
            # Prevent divide by zero
            self.X_std[self.X_std < 1e-9] = 1.0
            self.y_std[self.y_std < 1e-9] = 1.0

        self.current_state_dim = trajectories[0]['state_t'].shape[1]
                
        # Stats for normalizing the noisy state
        self.proprio_mean = self.X_mean[self.proprio_start_idx : self.proprio_end_idx]
        self.proprio_std = self.X_std[self.proprio_start_idx : self.proprio_end_idx]

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        traj_idx, frame_idx = self.indices[idx]
        traj = self.trajectories[traj_idx]
        start_idx, end_idx = frame_idx - self.k + 1, frame_idx + 1

        # --- 1. Get Clean Ground Truth Data ---
        # State Sequence (X_t) extraction remains unchanged
        if self.use_goal:
            full_state_sequence = np.concatenate([
                traj['state_t'][start_idx:end_idx],
                traj['goal_t'][start_idx:end_idx],
            ], axis=1)
        else:
            full_state_sequence = traj['state_t'][start_idx:end_idx]
        
        # --- Ground Truth Action (Y_t) calculation ---
        
        # Default DELTA action (used for joint_space delta and task_space delta)
        action_gt_delta_full = traj['action_t'][frame_idx]
        action_gt_delta = (action_gt_delta_full[:self.num_arm_actions] 
                           if self.arm_only else action_gt_delta_full)

        if self.action_space == 'delta':
            # This covers: Joint Delta (7D) and Task Delta (6D)
            action_gt = action_gt_delta

        elif self.action_space == 'absolute':
            # Action target is the ABSOLUTE POSE of the next state (S_t+1)
            
            # 1. Get the next state's full proprioception features
            q_next_full = traj['state_t'][frame_idx + 1] 
            q_next_proprio_full = q_next_full[self.proprio_start_idx : self.proprio_end_idx]
            
            # --- NEW LOGIC: ABSOLUTE TARGET CALCULATION ---
            if self.control_mode == 'joint_space':
                # ABSOLUTE JOINT POSE: Proprio features are already 7D joint angles (q_t+1)
                # Keep it 7D, slicing is just for arm_only if applicable
                action_gt = q_next_proprio_full[:self.num_arm_actions] if self.arm_only else q_next_proprio_full
            
            elif self.control_mode == 'task_space':
                # ABSOLUTE CARTESIAN POSE: Output must be 6D (3D Pos + 3D Rot Vec)
                
                # Arm slicing uses the first 7 DOFs of proprio (Pos+Quat)
                q_next_arm_7d = q_next_proprio_full[:7] 
                
                # 3D Position: [x, y, z]
                p_next = q_next_arm_7d[:3]
                
                # 4D Quaternion: [qx, qy, qz, qw]
                quat_next = q_next_arm_7d[3:7] 

                # Convert quaternion to 3D rotation vector (Axis-Angle form)
                from scipy.spatial.transform import Rotation as R
                r_next_vec = R.from_quat(quat_next).as_rotvec() # r_next_vec is 3D

                # Recombine to the 6D arm target
                action_gt_arm_6d = np.concatenate([p_next, r_next_vec], axis=0) # 6D
                
                if self.arm_only:
                    action_gt = action_gt_arm_6d
                else:
                    # Recombine 6D arm target with 16D hand joint targets
                    q_next_hand_16d = q_next_proprio_full[7:] 
                    action_gt = np.concatenate([action_gt_arm_6d, q_next_hand_16d], axis=0) # 22D total
            
            else:
                raise ValueError(f"Unknown control_mode: {self.control_mode}")
            
        # --- 2. Generate Unified Noise ---
        # We apply noise to the proprioception of the *current* (last) frame
        # NOTE: q_t_clean is still 7D (Pos+Quat) for task_space input!
        q_t_clean = traj['state_t'][frame_idx, self.proprio_start_idx : self.proprio_end_idx]
        
        # Calculate Correction Target (default is 0 if no noise)
        # action_gt_delta is the *FULL* delta action (7D or 6D)
        # We need action_gt_corr to match the shape of action_gt (22D or 23D etc.)
        action_corr_raw = np.zeros_like(action_gt) 
        q_t_noisy = q_t_clean.copy()

        if self.is_train:
            # A. General Observation Noise (applied to everything)
            # This helps robustness but doesn't change the target action
            full_state_norm = (full_state_sequence - self.X_mean) / self.X_std
            if self.obs_noise_std > 0:
                full_state_norm += np.random.normal(0, self.obs_noise_std, full_state_norm.shape)
            
            # B. Targeted Drift Noise (applied to proprioception)
            if np.random.rand() < self.DRIFT_PROB:
                sigma = self.drift_noise_std
            else:
                sigma = 0.0001 
            
            if sigma > 0:
                if self.control_mode == 'joint_space':
                    # Joint Space (7D) noise and correction remains delta (q_clean - q_noisy)
                    drift_noise = np.random.normal(0, sigma, q_t_clean.shape).astype(np.float32)
                    q_t_noisy = q_t_clean + drift_noise
                    action_corr_raw = q_t_clean - q_t_noisy # = -drift_noise (This is a 7D or 23D delta)
                    
                    # If action_gt is 7D, action_corr_raw is 7D. They match.
                    # If action_gt is 23D, action_corr_raw is 23D. They match.
                    
                elif self.control_mode == 'task_space' and PINOCCHIO_AVAILABLE:
                    # Task space (7D input) noise logic (Twist)
                    twist_noise = np.random.normal(0, sigma, 6).astype(np.float32)
                    drift_noise_hand = np.random.normal(0, sigma, 16).astype(np.float32)

                    # Apply noise to arm pose (Proprio is 7D (Pos+Quat))
                    T_clean = pin.XYZQUATToSE3(q_t_clean[0:7]) 
                    T_noise = pin.exp(twist_noise)
                    T_noisy = T_clean * T_noise
                    q_t_noisy_arm_7d = pin.SE3ToXYZQUAT(T_noisy)

                    # Apply noise to hand joints
                    q_t_noisy_hand_16d = q_t_clean[7:] + drift_noise_hand
                    q_t_noisy = np.concatenate([q_t_noisy_arm_7d, q_t_noisy_hand_16d], axis=0) # 23D noisy state
                    
                    # Correction: Twist needed to go from noisy -> clean
                    T_correction = T_noisy.inverse() * T_clean
                    action_corr_raw_arm_6d = pin.log(T_correction).vector # 6D twist correction
                    
                    # Hand correction is simple delta
                    action_corr_raw_hand_16d = q_t_clean[7:] - q_t_noisy_hand_16d 
                    
                    # The correction action must match the final target shape (action_gt)
                    if self.action_space == 'absolute':
                        # 1. Convert absolute ground truth (action_gt) back to 7D Pos+Quat for T_GT
                        # NOTE: This requires action_gt to have been calculated BEFORE this block.
                        p_gt = action_gt[:3]
                        r_gt_vec = action_gt[3:6]
                        from scipy.spatial.transform import Rotation as R
                        quat_gt = R.from_rotvec(r_gt_vec).as_quat() # [qx, qy, qz, qw]
                        q_next_arm_7d_gt = np.concatenate([p_gt, quat_gt], axis=0)
                        T_GT = pin.XYZQUATToSE3(q_next_arm_7d_gt)

                        # 2. Calculate Gap Twist (V_gap: from T_noisy -> T_GT)
                        T_gap = T_noisy.inverse() * T_GT
                        V_gap = pin.log(T_gap).vector # 6D Twist

                        # 3. Calculate Scaled/Corrected Twist (V_target)
                        # We use (1 - alpha) to scale the gap, so that alpha=1 means no correction.
                        V_target = V_gap * (1.0 - self.correction_alpha) 
                        
                        # 4. Compose Final Target Pose (T_final)
                        T_final = T_noisy * pin.exp(V_target)
                        
                        # 5. Convert T_final back to 6D Absolute Pose Target (Pos + RotVec)
                        q_t_final_arm_7d = pin.SE3ToXYZQUAT(T_final)
                        
                        p_final = q_t_final_arm_7d[:3]
                        quat_final = q_t_final_arm_7d[3:7] 

                        # Canonicalize quaternion before conversion to rotvec
                        if quat_final[3] < 0:
                            quat_final = -quat_final
                            
                        r_final_vec = R.from_quat(quat_final).as_rotvec()
                        
                        # Final corrected 6D arm target
                        action_gt_arm_6d_final = np.concatenate([p_final, r_final_vec], axis=0) # 6D
                        
                        # Append the uncorrected hand target
                        if not self.arm_only:
                            q_next_hand_16d = action_gt[6:] # Use the hand targets from the original action_gt
                            action_gt_final = np.concatenate([action_gt_arm_6d_final, q_next_hand_16d], axis=0)
                        else:
                            action_gt_final = action_gt_arm_6d_final

                        # IMPORTANT: Overwrite the target action (action_gt) with the geometrically corrected version
                        action_gt = action_gt_final
                        
                        # Set action_corr_raw to zeros, as the correction is now baked into action_gt
                        action_corr_raw = np.zeros_like(action_gt)
                    
                    elif self.action_space == 'delta':
                        # Target is 6D Twist + 16D Hand Delta
                        action_corr_raw = np.concatenate([action_corr_raw_arm_6d, action_corr_raw_hand_16d], axis=0)
                    else:
                         raise ValueError(f"Unknown action_space: {self.action_space}")
                
                # C. Inject the specific drift into the normalized state vector
                # q_t_noisy is 7D (or 23D). This keeps the input features consistent.
                q_t_noisy_norm = (q_t_noisy - self.proprio_mean) / self.proprio_std
                
                # Replace proprio features in the LAST frame of the stack
                full_state_norm[-1, self.proprio_start_idx : self.proprio_end_idx] = q_t_noisy_norm

        else:
            # Validation/Test: Just normalize clean data
            full_state_norm = (full_state_sequence - self.X_mean) / self.X_std

        # --- 3. Calculate Unified Target Action ---
        # Target = Normalized_Demonstrator_Action + Clipped_Normalized_Correction
        
        # Normalize Demonstrator Action
        action_gt_norm = (action_gt - self.y_mean) / self.y_std 
        
        # Normalize Correction Action (Now using self.y_std, which is 22D or 7D etc.)
        action_corr_norm = action_corr_raw / self.y_std

        # --- Revised Clipping: Smooth correction proportional to the error (P gain) ---
        action_corr_clamped = action_corr_norm * self.correction_alpha
        
        # Sum them up
        final_action_target = action_gt_norm + action_corr_clamped
        
        # --- 4. Flatten and Return ---
        state_output = full_state_norm.flatten() if self.flatten else full_state_norm

        return torch.from_numpy(state_output).float(), torch.from_numpy(final_action_target).float()

class JointWeightedMSELoss(nn.Module):
    def __init__(self, action_weights, control_mode='joint_space', action_space='delta', is_arm_only=False):
            """
            Calculates Mean Squared Error on NORMALIZED actions, weighted by importance,
            and uses a minimal distance metric for Cartesian Orientation (RotVec).

            Args:
                action_weights (torch.Tensor): Vector of weights, one per joint.
                control_mode (str): 'joint_space' or 'task_space'.
                action_space (str): 'delta' or 'absolute'.
                is_arm_only (bool): If True, output does not include hand joints.
            """
            super().__init__()
            self.action_weights = action_weights.float().to(action_weights.device)
            self.mse_loss = nn.MSELoss(reduction='none') 
            
            self.is_task_absolute = (control_mode == 'task_space') and (action_space == 'absolute')
            self.is_arm_only = is_arm_only
            
            # --- Constants for Task Space Orientation Slicing ---
            # The first 3 dimensions are position (or joint angles, if joint_space)
            self.POS_END_IDX = 3 
            
            # RotVec starts at index 3 and ends at index 6 (for the 6D arm output)
            if self.is_task_absolute:
                self.ORI_START_IDX = 3
                self.ORI_END_IDX = 6
                # Hand/Remaining DOFs start after the arm's 6D output
                self.HAND_START_IDX = 6

    def forward(self, pred_norm, target_norm):
        """
        Args:
            pred_norm (torch.Tensor): Model output (normalized action).
            target_norm (torch.Tensor): Ground-truth target action (normalized).
        """
        
        # Standard squared error for all components
        squared_error = self.mse_loss(pred_norm, target_norm) # Shape: (B, D)

        if self.is_task_absolute:
            # 1. Separate the Rotation Vector component (R_x, R_y, R_z)
            pred_ori_norm = pred_norm[:, self.ORI_START_IDX : self.ORI_END_IDX]
            target_ori_norm = target_norm[:, self.ORI_START_IDX : self.ORI_END_IDX]
            
            # 2. Calculate the "Flipped" Error for the Rotation Vector
            # The rotation vector r and -r represent rotations around the same axis 
            # by angles theta and -theta. In the normalized space, this is equivalent to
            # comparing r_pred vs r_target AND r_pred vs -r_target (the 2*pi difference).
            
            # Error 1: Standard (r_pred - r_target)^2
            sq_err_std = (pred_ori_norm - target_ori_norm)**2
            
            # Error 2: Flipped (r_pred - (-r_target))^2 = (r_pred + r_target)^2
            # This handles the case where r_target is near +pi and r_pred is near -pi (or vice-versa).
            sq_err_flip = (pred_ori_norm + target_ori_norm)**2 
            
            # 3. Use the MINIMUM of the two squared errors for each orientation DOF
            min_sq_err_ori = torch.min(sq_err_std, sq_err_flip)
            
            # 4. Inject the corrected minimum squared error back into the full error tensor
            
            # Position/Other Arm DOFs (Pos in task space)
            error_pos = squared_error[:, :self.POS_END_IDX]

            # Hand/Remaining DOFs (if not arm_only)
            if not self.is_arm_only and pred_norm.shape[1] > self.HAND_START_IDX:
                error_hand = squared_error[:, self.HAND_START_IDX:]
                # Concatenate Position error, Corrected Orientation error, and Hand error
                corrected_error = torch.cat([error_pos, min_sq_err_ori, error_hand], dim=1)
            else:
                # Concatenate Position error and Corrected Orientation error only
                corrected_error = torch.cat([error_pos, min_sq_err_ori], dim=1)
            
            # Sanity check: ensure the dimension hasn't changed
            assert corrected_error.shape == squared_error.shape, "Error dimension mismatch after correction."
            squared_error = corrected_error
        
        # 5. Apply Joint Weights (Standard for all modes)
        weighted_squared_error = squared_error * self.action_weights.view(1, -1)

        # 6. Compute Mean Loss
        return torch.mean(weighted_squared_error)

# --- CUSTOM LOSS FUNCTION (Implements NIDL and Static Joint Weighting) ---
class JointWeightedDenormalizedMSELoss(nn.Module):
    def __init__(self, y_mean, y_std, action_weights):
        """
        Calculates MSE on REAL (unnormalized) action space, weighted by joint importance.
        
        Args:
            y_mean (torch.Tensor): Mean vector used for action normalization.
            y_std (torch.Tensor): Standard deviation vector used for action normalization.
            action_weights (torch.Tensor): Vector of weights, one per joint.
        """
        super().__init__()
        # Ensure constants and weights are float tensors and on the correct device
        self.y_mean = y_mean.float().to(y_mean.device)
        self.y_std = y_std.float().to(y_std.device)
        self.action_weights = action_weights.float().to(y_mean.device)
        
        # Use reduction='none' so we can apply weights before taking the mean
        self.mse_loss = nn.MSELoss(reduction='none') 

    def forward(self, pred_norm, target_norm):
        """
        Args:
            pred_norm (torch.Tensor): Model output (normalized action).
            target_norm (torch.Tensor): Ground-truth target action (normalized).
        """
        
        # 1. Denormalize Prediction (Model Output)
        # This converts the prediction back to physical units (radians/meters)
        pred_real = (pred_norm * self.y_std) + self.y_mean

        # 2. Denormalize Target (Ground Truth)
        # Since the DataLoader only returned normalized target, we must denormalize 
        # it here to calculate the error in the physical space.
        target_real = (target_norm * self.y_std) + self.y_mean

        # 3. Calculate Un-reduced Squared Error in REAL space (NIDL)
        squared_error = self.mse_loss(pred_real, target_real)

        # 4. Apply Joint Weights
        # The weight vector is broadcast across the batch dimension
        weighted_squared_error = squared_error * self.action_weights

        # 5. Compute Mean Loss across all dimensions (batch and joints)
        return torch.mean(weighted_squared_error)
    
# ===================================================================
# === UTILITY FUNCTIONS ===
# ===================================================================
def set_seed(seed=42):
    np.random.seed(seed); random.seed(seed); torch.manual_seed(seed)
    if torch.cuda.is_available(): torch.cuda.manual_seed_all(seed)

def build_model(model_type, input_dim, output_dim, **kwargs):
    if model_type == "mlp":
        return MLPPolicy(input_dim, output_dim, width=kwargs.get("width", 256))
    elif model_type == "lstm":
        return LSTMPolicy(input_dim, output_dim, hidden_dim=kwargs.get("hidden_dim", 256), num_layers=kwargs.get("num_layers", 2))
    elif model_type == "gru":
        return GRUPolicy(input_dim, output_dim, hidden_dim=kwargs.get("hidden_dim", 256), num_layers=kwargs.get("num_layers", 2))
    elif model_type == "transformer":
        return TransformerPolicy(
            input_dim, output_dim,
            num_heads=kwargs.get("num_heads", 4),
            hidden_dim=kwargs.get("hidden_dim", 256),
            num_layers=kwargs.get("num_layers", 2)
        )
    else:
        raise ValueError(f"Unknown model_type '{model_type}'")

# ===================================================================
# === MAIN FUNCTION ===
# ===================================================================
def main():
    parser = argparse.ArgumentParser(description="Interactively train a policy model.")
    # All arguments are now optional and have defaults. They can override interactive selections.
    parser.add_argument("--dataset_pkl", type=str, help="Path to the .pkl dataset file.")
    parser.add_argument("--output_dir", type=str, default=str(paths.POLICY_MODELS_DIR), help="Directory to save models.")
    parser.add_argument("--model_type", type=str, choices=["mlp", "lstm", "gru", "transformer"], help="Model architecture.")
    parser.add_argument("--frame_stack", type=int, help="Number of frames to stack (K).")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--batch_size", type=int, default=64)
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--split_ratio", type=float, default=0.85)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--arm_only", action=argparse.BooleanOptionalAction, help="Train a policy for arm joints only.")
    parser.add_argument("--validation", action=argparse.BooleanOptionalAction, help="Use validation loss during training.")
    parser.add_argument("--config_file", type=str, help="Path to the configuration YAML used to build the dataset.")
    parser.add_argument("--num_arm_joints", type=int, default=7, help="Number of arm joints.")
    parser.add_argument("--adaptive_drift_weight", action=argparse.BooleanOptionalAction, help="Use an adaptive weight for the drift regularizer loss, if using it.")
    parser.add_argument("--proprio_only", action=argparse.BooleanOptionalAction, help="Train on proprioception features only (for debug or testing).")

    # Model-specific hyperparameters
    parser.add_argument("--width", type=int, help="Width of hidden layers for MLP.")
    parser.add_argument("--hidden_dim", type=int, help="Hidden dimension for LSTM/GRU/Transformer.")
    parser.add_argument("--num_layers", type=int, help="Number of layers for LSTM/GRU/Transformer.")
    parser.add_argument("--num_heads", type=int, help="Number of attention heads for Transformer.")

    args = parser.parse_args()

    config_choices = find_config_files(paths.CONFIG_DIR)
    if not config_choices:
        logging.error(f"No .yaml config files found in {paths.CONFIG_DIR}. Exiting."); return

    try:
        answers = {}
        # --- Step 1: General Configuration ---
        base_questions = []
        if args.dataset_pkl is None:
            pkl_choices = find_pkl_files(paths.PROCESSED_DATA_DIR)
            if not pkl_choices: logging.error(f"No .pkl files found in {paths.PROCESSED_DATA_DIR}."); return
            base_questions.append(inquirer.List('dataset_pkl', message="Select the dataset to train on", choices=pkl_choices))
        base_questions.append(inquirer.List('config_file',
                message="Select the configuration file used to build this dataset",
                choices=config_choices))
        if args.arm_only is None:
            base_questions.append(inquirer.Confirm('arm_only', message="Train in ARM-ONLY mode?", default=False))
        if args.validation is None:
            base_questions.append(inquirer.Confirm('validation', message="Use validation loss during training?", default=False))
        if base_questions:
            answers.update(inquirer.prompt(base_questions) or {})
        
        # --- Step 2: Model Selection ---
        if args.model_type is None:
            model_question = [inquirer.List('model_type', message="Select model architecture", choices=['mlp', 'lstm', 'gru', 'transformer'], default='mlp')]
            answers.update(inquirer.prompt(model_question) or {})

        # Use the answer from the prompt if the arg wasn't provided
        model_type = args.model_type or answers.get('model_type')

        # --- Step 3: Model-Specific Hyperparameters ---
        model_specific_questions = []
        if model_type == 'mlp' and args.width is None:
            model_specific_questions.append(inquirer.Text('width', message="Enter MLP width", default='512'))
            model_specific_questions.append(inquirer.Text('num_layers', message="Enter MLP number of layers", default='2'))

        if model_type in ['lstm', 'gru'] and args.hidden_dim is None:
            model_specific_questions.append(inquirer.Text('hidden_dim', message=f"Enter {model_type.upper()} hidden dimension", default='256'))
            model_specific_questions.append(inquirer.Text('num_layers', message=f"Enter {model_type.upper()} number of layers", default='2'))

        if model_type == 'transformer' and args.hidden_dim is None:
            model_specific_questions.append(inquirer.Text('hidden_dim', message="Enter Transformer hidden (feedforward) dimension", default='256'))
            model_specific_questions.append(inquirer.Text('num_layers', message="Enter Transformer number of encoder layers", default='2'))
            model_specific_questions.append(inquirer.Text('num_heads', message="Enter Transformer number of attention heads", default='4'))
        
        if model_specific_questions:
            answers.update(inquirer.prompt(model_specific_questions) or {})
        
        # --- Step 4: Final Training Parameters ---
        final_questions = []
        if args.frame_stack is None:
            default_k = '10' if model_type != 'mlp' else '3'
            final_questions.append(inquirer.Text('frame_stack', message="Enter frame stack size (K)", default=default_k))
        if args.lr == 1e-4: # If it's the default, ask
             final_questions.append(inquirer.Text('lr', message="Enter learning rate", default='1e-4'))
        if args.batch_size == 64:
             final_questions.append(inquirer.Text('batch_size', message="Enter batch size", default='64'))
        
        if final_questions:
             answers.update(inquirer.prompt(final_questions) or {})


        # --- Combine args and answers ---
        final_args = argparse.Namespace(**vars(args))
        for key, value in answers.items():
            if getattr(final_args, key) is None:
                try: # Try to convert to float/int first
                    if '.' in str(value): value = float(value)
                    else: value = int(value)
                except (ValueError, TypeError): pass
                setattr(final_args, key, value)

        # Config file
        config_path_rel = final_args.config_file
        config_path_abs = paths.WORKSPACE_ROOT / config_path_rel
        with open(config_path_abs, 'r') as f:
            config = yaml.safe_load(f)
        logging.info(f"Loaded configuration from {config_path_abs}")
        control_mode = config.get('control_mode', 'joint_space')
        logging.info(f"Control mode: {control_mode}")
        action_space = config.get("action_space", "delta")
        logging.info(f"Action space: {action_space}")
        use_goal = config.get("use_goal", False)
        logging.info(f"Use goal conditioning: {use_goal}")
        use_drift_regularizer = config.get("use_drift_regularizer", False)
        logging.info(f"Use drift regularizer: {use_drift_regularizer}")
        drift_loss_weight = config.get("drift_loss_weight", 0.1)
        drift_noise_std = config.get("drift_noise_std", 0.05)

        if use_drift_regularizer:
            logging.info(f"     Drift loss weight: {drift_loss_weight}")
            logging.info(f"     Drift noise std: {drift_noise_std}")

        joint_indices = config.get("joint_indices", [24, 31])
        obs_noise_std_train = config.get("obs_noise_std_train", 0.01)
        obs_noise_std_val = config.get("obs_noise_std_val", 0.0)

        logging.info(f"Observation noise std (train): {obs_noise_std_train}")
        if final_args.validation:
            logging.info(f"Observation noise std (val): {obs_noise_std_val}")

        if not final_args.dataset_pkl: logging.info("No dataset selected. Exiting."); return
        logging.info(f"Final training configuration: {final_args}")

        # --- 3. Run the Training Logic ---
        device = "cuda" if torch.cuda.is_available() else "cpu"
        logging.info(f"Using device: {device}")
        set_seed(final_args.seed)

        #  Use the final_args object to get the output directory
        output_dir = Path(final_args.output_dir); debug_dir = output_dir / "debug"
        output_dir.mkdir(parents=True, exist_ok=True); debug_dir.mkdir(exist_ok=True)

        with open(paths.WORKSPACE_ROOT / final_args.dataset_pkl, "rb") as f:
            all_trajectories = pickle.load(f)

        # Train Test split
        random.seed(final_args.seed); random.shuffle(all_trajectories)

        if final_args.validation:
            split_index = int(len(all_trajectories) * final_args.split_ratio)
            train_trajectories = all_trajectories[:split_index]
            val_trajectories = all_trajectories[split_index:]
            logging.info(f"Loaded {len(all_trajectories)} trajectories: {len(train_trajectories)} train, {len(val_trajectories)} val.")
        else:
            logging.info(f"Loaded {len(all_trajectories)} trajectories: Using for training only.")

        if final_args.arm_only:
            if control_mode == 'joint_space':
                logging.info(f"ARM-ONLY mode enabled. Slicing ACTIONS to the first {final_args.num_arm_joints} joints.")
                n = final_args.num_arm_joints
                
                for traj in all_trajectories:
                    # We ONLY slice the action vector (the output of the policy).
                    traj['action_t'] = traj['action_t'][:, :n]

                    # We DO NOT slice 'joints_t' or 'goal_t'.
                    # The policy's input (the state) should contain the full 23 joints.
            else:
                logging.info(f"ARM-ONLY mode enabled. Slicing ACTIONS to the arm 6D pose.")
                
                for traj in all_trajectories:
                    # We ONLY slice the action vector (the output of the policy).
                    traj['action_t'] = traj['action_t'][:, :6]

        logging.info("Preparing datasets and dataloaders...")

        if final_args.validation:
            logging.info(f"Train dataset action shape example: {train_trajectories[0]['action_t'].shape}")
            logging.info(f"Val dataset action shape example: {val_trajectories[0]['action_t'].shape}")

            is_sequence_model = final_args.model_type in ["lstm", "gru", "transformer"]
            train_dataset = TrajectoryFrameStackDataset(
                train_trajectories,
                final_args.frame_stack,
                config,
                final_args.arm_only,
                flatten=(not is_sequence_model),
                is_train=True,
                obs_noise_std=obs_noise_std_train,
                drift_noise_std=drift_noise_std,
                # return_drift_data=use_drift_regularizer,
                # proprio_only=final_args.proprio_only
            )

            val_dataset = TrajectoryFrameStackDataset(
                val_trajectories,
                final_args.frame_stack,
                config,
                final_args.arm_only,
                norm_stats=(train_dataset.X_mean, train_dataset.X_std, train_dataset.y_mean, train_dataset.y_std),
                flatten=(not is_sequence_model),
                is_train=False,
                obs_noise_std=obs_noise_std_val,
                # return_drift_data=use_drift_regularizer,
                # proprio_only=final_args.proprio_only
            )

            if len(train_dataset) == 0:
                logging.error("Training dataset is empty!"); return

            train_loader = DataLoader(train_dataset, batch_size=final_args.batch_size, shuffle=True, num_workers=TARGET_WORKERS,  pin_memory=True)
            val_loader = DataLoader(val_dataset, batch_size=final_args.batch_size * 2, num_workers=TARGET_WORKERS,  pin_memory=True)

        else:
            logging.info(f"Train dataset action shape example: {all_trajectories[0]['action_t'].shape}")
            is_sequence_model = final_args.model_type in ["lstm", "gru", "transformer"]

            train_dataset = TrajectoryFrameStackDataset(
                all_trajectories,
                final_args.frame_stack,
                config,
                final_args.arm_only,
                flatten=(not is_sequence_model),
                is_train=True,
                obs_noise_std=obs_noise_std_train,
                drift_noise_std=drift_noise_std,
                # return_drift_data=use_drift_regularizer,
                # proprio_only=final_args.proprio_only
            )

            if len(train_dataset) == 0:
                logging.error("Training dataset is empty!"); return

            train_loader = DataLoader(train_dataset, batch_size=final_args.batch_size, shuffle=True, num_workers=TARGET_WORKERS,  pin_memory=True)

        # --- Model setup (before the loop) ---
        single_frame_dim = train_dataset.X_mean.shape[0]
        output_dim = train_dataset.y_mean.shape[0]
        input_dim = single_frame_dim * final_args.frame_stack if not is_sequence_model else single_frame_dim

        model = build_model(
            final_args.model_type, input_dim, output_dim,
            width=final_args.width,
            num_heads=final_args.num_heads,
            hidden_dim=final_args.hidden_dim,
            num_layers=final_args.num_layers
        ).to(device)
        optimizer = optim.Adam(model.parameters(), lr=final_args.lr, weight_decay=1e-3)

        W_HIGH = config.get("joint_high_weight", 2.0)
        W_LOW = config.get("joint_low_weight", 1.0)
        action_weights_np = action_weights_np = np.full((output_dim,), W_HIGH, dtype=np.float32)
        # Set low priority for hand joints (last 16 indices)
        action_weights_np[-16:] = W_LOW

        # INDEX_START, INDEX_END = 7, 11  # Joints 7, 8, 9, 10
        # THUMB_START, THUMB_END = 19, 23 # Joints 19, 20, 21, 22

        # if not final_args.arm_only:
        #     # Index
        #     action_weights_np[INDEX_START:INDEX_END] = W_HIGH
        #     # Thumb
        #     action_weights_np[THUMB_START:THUMB_END] = W_HIGH

        action_weights_t = torch.from_numpy(action_weights_np).float().to(device)

        logging.info(f"Action Loss Weights Status (Total {output_dim} joints):")
        logging.info(f"  Arm (0-{final_args.num_arm_joints-1}): W={W_HIGH}x")
        # if not final_args.arm_only:
        #     logging.info(f"  Hand Index ({INDEX_START}-{INDEX_END-1}): W={W_HIGH}x")
        #     logging.info(f"  Hand Thumb ({THUMB_START}-{THUMB_END-1}): W={W_HIGH}x")
        #     logging.info(f"  Other Hand Joints: W={W_LOW}x")

        # --- 2. Instantiate the Custom Loss Function ---
        loss_fn = JointWeightedMSELoss(
            action_weights=action_weights_t,
            control_mode=control_mode,
            action_space=action_space,
            is_arm_only=final_args.arm_only
        )

        y_mean_t = torch.from_numpy(train_dataset.y_mean).float().to(device)
        y_std_t  = torch.from_numpy(train_dataset.y_std).float().to(device)

        logging.info(f"Training with Frame Stacking (K={final_args.frame_stack}) for '{final_args.model_type.upper()}' model.")
        logging.info(f"  Input Dim: {input_dim}, Output Dim: {output_dim}")

        history = {"train_loss": [], "val_loss": []}
        best_val_loss = float('inf')
        best_train_loss = float('inf')
        patience_counter = 0
        model_save_path = output_dir / f"policy_{final_args.model_type}_{control_mode}_{'arm' if final_args.arm_only else 'hand'}.pt"
        i = 1
        while os.path.exists(model_save_path):
            i+=1
            model_save_path = output_dir / f"policy_{final_args.model_type}_{control_mode}_{'arm' if final_args.arm_only else 'hand'}{i}.pt"

        # --- ADAPTIVE WEIGHTING SETUP ---
        global_step = 0
        drift_weight_adaptive = drift_loss_weight # Initialize with config value
        EMA_ALPHA = 0.99 
        EMA_EPSILON = 1e-6 # For stable division
        DRIFT_CLIP_TAU = 10.0

        if final_args.adaptive_drift_weight and use_drift_regularizer:
            logging.info(f"--- Using Adaptive Loss Weighting (EMA Alpha: {EMA_ALPHA}) ---")
            # Initialize EMA with small positive values, possibly based on the first batch losses
            ema_bc_loss = None 
            ema_drift_loss = None
            
        # --- Training loop ---
        try:
            for epoch in range(final_args.epochs):
                model.train()
                total_train_loss = 0.0

                for batch in train_loader:
                    state_norm, action_target_norm = batch
                    state_norm, action_target_norm = state_norm.to(device), action_target_norm.to(device)
                    
                    optimizer.zero_grad()

                    # 1. Forward Pass
                    pred_norm = model(state_norm)

                    # 2. Loss Calculation
                    # The target already contains (BC + Clipped_Correction)
                    loss = loss_fn(pred_norm, action_target_norm)

                    loss.backward()
                    optimizer.step()
                    total_train_loss += loss.item()

                avg_train_loss = total_train_loss / len(train_loader)
                history["train_loss"].append(avg_train_loss)

                # --- Validation phase ---
                if final_args.validation:
                    model.eval()
                    total_val_loss = 0.0
                    with torch.no_grad():
                        for batch in val_loader:
                            state_norm, action_target_norm = batch
                            state_norm, action_target_norm = state_norm.to(device), action_target_norm.to(device)
                            
                            pred_norm = model(state_norm)
                            loss = loss_fn(pred_norm, action_target_norm)
                            total_val_loss += loss.item()

                    avg_val_loss = total_val_loss / len(val_loader)
                    history["val_loss"].append(avg_val_loss)
                    logging.info(f"Epoch {epoch+1:03d}/{final_args.epochs} | Train: {avg_train_loss:.6f} | Val: {avg_val_loss:.6f}")
                else:
                    logging.info(f"Epoch {epoch+1:03d}/{final_args.epochs} | Train Loss: {avg_train_loss:.6f}")


                # --- Model saving logic (unchanged) ---
                if (final_args.validation and avg_val_loss < best_val_loss) or (not final_args.validation and avg_train_loss < best_train_loss):
                    if final_args.validation:
                        best_val_loss = avg_val_loss
                    else:
                        best_train_loss = avg_train_loss

                    model_hyperparams = {
                        "width": final_args.width,
                        "hidden_dim": final_args.hidden_dim,
                        "num_layers": final_args.num_layers,
                        "num_heads": final_args.num_heads
                    }

                    torch.save({
                        "state_dict": model.state_dict(),
                        "model_type": final_args.model_type,
                        "input_dim": input_dim, "output_dim": output_dim,
                        "model_hyperparams": model_hyperparams,
                        "X_mean": train_dataset.X_mean, "X_std": train_dataset.X_std,
                        "y_mean": train_dataset.y_mean, "y_std": train_dataset.y_std,
                        "frame_stack": final_args.frame_stack,
                        "validation": final_args.validation,
                        "control_mode": control_mode,
                        "training_config": config,
                        "best_loss": best_val_loss if final_args.validation else best_train_loss,
                        "epoch": epoch + 1,
                        "arm_only": final_args.arm_only,
                        "num_arm_joints": final_args.num_arm_joints
                    }, model_save_path)

                    msg = f"  -> New best model saved to {model_save_path} "
                    msg += f"(Val Loss: {best_val_loss:.6f})" if final_args.validation else f"(Train Loss: {best_train_loss:.6f})"
                    logging.info(msg)
                    patience_counter = 0
                else:
                    patience_counter += 1

                if patience_counter >= final_args.patience:
                    logging.info(f"Early stopping: no improvement for {final_args.patience} epochs.")
                    break

        except KeyboardInterrupt:
            logging.info("Training interrupted.")
            
        if final_args.validation:
            logging.info(f"🏁 Training complete. Best validation loss: {best_val_loss:.6f}")
        else:
            logging.info(f"🏁 Training complete. Best training loss: {best_train_loss:.6f}")

        plt.figure(figsize=(10, 5))
        plt.plot(history["train_loss"], label="Training Loss")

        if final_args.validation:
            plt.plot(history["val_loss"], label="Validation Loss")
            plt.xlabel("Epoch"); plt.ylabel("MSE Loss"); plt.title(f"Training and Validation Loss ({final_args.model_type})")
        else:
            plt.xlabel("Epoch"); plt.ylabel("MSE Loss"); plt.title(f"Training Loss ({final_args.model_type})")

        plt.legend(); plt.grid(True)
        plot_save_path = debug_dir / f"loss_curve_{final_args.model_type}_{control_mode}.png"
        plt.savefig(plot_save_path)
        logging.info(f"📉 Saved loss curve to {plot_save_path}")

    except (KeyboardInterrupt, TypeError) as e:
        logging.error(f"An error occurred: {e}", exc_info=True) # Log the full traceback
        logging.info("\nTraining configuration cancelled or failed.")
        return

if __name__ == "__main__":
    main()

