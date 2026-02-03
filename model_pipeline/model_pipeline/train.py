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
from scipy.spatial.transform import Rotation as R

from model_pipeline.utils import find_config_files
from model_pipeline import paths
from model_pipeline.utils import find_pkl_files

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except:
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
            dropout=0.2
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
        self.obs_noise_std = obs_noise_std 
        self.drift_noise_std = drift_noise_std

        # --- Config options ---
        self.arm_only = arm_only
        self.control_mode = config.get('control_mode', 'joint_space')
        self.action_space = config.get('action_space', 'delta') # 'delta' or 'absolute'
        self.use_goal = config.get('use_goal', False)

        # --- Dimensions ---
        # Proprioception (Input) is ALWAYS 7 for arm (Pos + Quat) or 7 joints
        self.proprio_arm_dim = 7 
        
        # Action (Output) dimensions depend on mode
        if self.control_mode == 'task_space':
            if self.action_space == 'absolute':
                self.num_arm_actions = 9  # 3 Pos + 6 Rot (Ortho6D)
            else:
                self.num_arm_actions = 6  # 6 Twist (Delta)
        else:
            self.num_arm_actions = 7      # 7 Joints

        # --- Constants for Unified Drift Logic ---
        self.DRIFT_PROB = config.get('drift_prob', 0.2)
        self.DRIFT_CLIP_TAU = config.get('drift_clip_tau', 4)
        self.correction_alpha = config.get('correction_alpha', 0.1)

        # --- Pre-calculate Proprioception Slices ---
        TACTILE_DIM = 24
        self.proprio_start_idx = TACTILE_DIM 
        
        # We drift the input features (proprio). 
        # For task space input, proprio is 7D (Pos+Quat). For joint space, it's 7D (Joints).
        drift_dim = self.proprio_arm_dim if self.arm_only else 23 
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
            # 1. State Stats
            if self.use_goal:
                X_unstacked = np.concatenate([
                    np.concatenate([t['state_t'], t['goal_t']], axis=1)
                    for t in trajectories
                ], axis=0)
            else:
                X_unstacked = np.concatenate([t['state_t'] for t in trajectories], axis=0)

            # 2. Action Stats
            if self.action_space == 'delta':
                y_unstacked_full = np.concatenate([t['action_t'] for t in trajectories], axis=0)
                
            elif self.action_space == 'absolute':
                y_data_list = []
                
                for traj in trajectories:
                    # Input Proprio: 7D Arm (Pos+Quat) + Hand
                    q_next_proprio_full = traj['state_t'][1:, self.proprio_start_idx : self.proprio_end_idx]
                    
                    if self.control_mode == 'joint_space':
                        y_data_list.append(q_next_proprio_full)
                        
                    elif self.control_mode == 'task_space':
                        # CONVERSION: 7D Pose -> 9D Ortho6D Action
                        y_traj_list = []
                        for i in range(q_next_proprio_full.shape[0]):
                            q_next_7d = q_next_proprio_full[i, :7]
                            q_next_hand_16d = q_next_proprio_full[i, 7:]
                            
                            # Helper to convert to 9D
                            action_gt_arm_9d = self._convert_7d_to_9d(q_next_7d)
                            
                            # Recombine with hand
                            action_gt_full = np.concatenate([action_gt_arm_9d, q_next_hand_16d], axis=0)
                            y_traj_list.append(action_gt_full)
                            
                        y_data_list.append(np.stack(y_traj_list, axis=0))

                y_unstacked_full = np.concatenate(y_data_list, axis=0)
            
            else:
                raise ValueError(f"Unknown action_space: {self.action_space}")
                
            # Final slicing based on arm_only
            # y_unstacked_full is now 25D (9+16) for task space absolute, or 23D (7+16) for joint
            y_unstacked = (y_unstacked_full[:, :self.num_arm_actions]
                           if self.arm_only else y_unstacked_full)

            self.X_mean = X_unstacked.mean(axis=0)
            self.X_std  = X_unstacked.std(axis=0)
            self.y_mean = y_unstacked.mean(axis=0)
            self.y_std  = y_unstacked.std(axis=0)
            
            # Robustness
            self.X_std[self.X_std < 1e-6] = 1.0
            self.y_std[self.y_std < 1e-6] = 1.0

        self.current_state_dim = trajectories[0]['state_t'].shape[1]
                
        # Stats for normalizing the noisy state (Input features are Proprio)
        self.proprio_mean = self.X_mean[self.proprio_start_idx : self.proprio_end_idx]
        self.proprio_std = self.X_std[self.proprio_start_idx : self.proprio_end_idx]

    def _convert_7d_to_9d(self, pose_7d):
        """
        Helper: Converts [x,y,z, qx,qy,qz,qw] -> [x,y,z, r11,r21,r31,r12,r22,r32]
        """
        pos = pose_7d[:3]
        quat = pose_7d[3:]
        # Convert Quat -> Rotation Matrix
        rot_mat = R.from_quat(quat).as_matrix() # 3x3
        # Flatten first two columns (Ortho6D)
        r6d = rot_mat[:, :2].T.flatten() # 6D
        return np.concatenate([pos, r6d])

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        traj_idx, frame_idx = self.indices[idx]
        traj = self.trajectories[traj_idx]
        
        # Define window
        start_idx = frame_idx - self.k + 1
        end_idx = frame_idx + 1 # Current frame index
        
        # --- 1. Get Clean Data ---
        # Sequence of states (History) [K, State_Dim]
        state_seq_clean = traj['state_t'][start_idx:end_idx].copy()
        
        # Current Clean State (Single Frame) [State_Dim]
        q_t_clean_full = traj['state_t'][frame_idx].copy()
        
        # Expert Next State (Single Frame) [State_Dim]
        q_next_clean_full = traj['state_t'][frame_idx + 1].copy()

        # Initialize Noisy Sequence
        state_seq_noisy = state_seq_clean.copy()
        target_action_raw = None 

        # --- 2. Apply Sequence-Wide Drift (Train Only) ---
        if self.is_train:
            # Decide drift
            apply_drift = np.random.rand() < self.DRIFT_PROB
            # Use small epsilon if not drifting to avoid divide-by-zero or empty logic
            sigma = self.drift_noise_std if apply_drift else 0.00001
            
            p_start, p_end = self.proprio_start_idx, self.proprio_end_idx
            
            # --- JOINT SPACE LOGIC ---
            if self.control_mode == 'joint_space':
                # Generate ONE drift vector for the whole sequence
                # Shape: (23,) if arm+hand, or (7,) if arm only
                drift_vector = np.random.normal(0, sigma, p_end - p_start).astype(np.float32)
                
                # Apply drift to ALL frames in the sequence (Broadcasting (K, 23) += (23,))
                state_seq_noisy[:, p_start:p_end] += drift_vector
                
                # --- Calculate Target Action ---
                # Slice the clean vectors to get just the proprio parts we care about
                q_next_proprio = q_next_clean_full[p_start:p_end]
                q_curr_proprio = q_t_clean_full[p_start:p_end]
                
                if self.action_space == 'absolute':
                    # Target = Expert_Next + (1 - alpha) * Error
                    # We want to be at Expert_Next, but we accept keeping some of the current drift
                    target_proprio = q_next_proprio + (1.0 - self.correction_alpha) * drift_vector
                else: 
                    # Delta Mode
                    # Standard Delta = Expert_Next - Current_Clean
                    # Corrective Delta = Standard_Delta - alpha * Drift
                    clean_delta = q_next_proprio - q_curr_proprio
                    target_proprio = clean_delta - (self.correction_alpha * drift_vector)

                # Filter for Arm Only if needed
                target_action_raw = target_proprio[:self.num_arm_actions] if self.arm_only else target_proprio

            # --- TASK SPACE LOGIC ---
            elif self.control_mode == 'task_space' and PINOCCHIO_AVAILABLE:
                # 1. Generate Drifts
                twist_drift = np.random.normal(0, sigma, 6).astype(np.float32) # 6D
                hand_drift = np.random.normal(0, sigma, 16).astype(np.float32) # 16D
                
                # Precompute Drift Matrix
                T_drift = pin.exp(twist_drift)

                # 2. Apply Drift to Sequence History
                # We iterate because SE(3) isn't a simple addition
                for i in range(self.k):
                    # Arm: Extract 7D pose -> SE3 -> Apply Drift -> Back to 7D
                    q_i_arm = state_seq_clean[i, p_start : p_start+7]
                    T_i_clean = pin.XYZQUATToSE3(q_i_arm)
                    T_i_noisy = T_i_clean * T_drift # Systematic sensor bias
                    state_seq_noisy[i, p_start : p_start+7] = pin.SE3ToXYZQUAT(T_i_noisy)
                    
                    # Hand: Linear addition
                    if not self.arm_only:
                        state_seq_noisy[i, p_start+7 : p_end] += hand_drift

                # 3. Calculate Target Action (Single Frame)
                # We need the "Current Noisy" state to calculate delta or correction
                T_curr_noisy = pin.XYZQUATToSE3(state_seq_noisy[-1, p_start : p_start+7])
                T_next_clean = pin.XYZQUATToSE3(q_next_clean_full[p_start : p_start+7])
                
                # Damped Target Pose: Expert * exp( (1-alpha) * drift )
                # Effectively: "Go to expert, but offset by 90% of the current error"
                T_target_damped = T_next_clean * pin.exp(twist_drift * (1.0 - self.correction_alpha))
                
                # Arm Action Part
                if self.action_space == 'absolute':
                    # Absolute: Output the Damped Target Pose (as Ortho6D)
                    q_target_7d = pin.SE3ToXYZQUAT(T_target_damped)
                    action_arm = self._convert_7d_to_9d(q_target_7d)
                else: 
                    # Delta: Twist from Current Noisy -> Damped Target
                    # "How do I move from where I THINK I am, to where I want to be?"
                    action_arm = pin.log(T_curr_noisy.actInv(T_target_damped)).vector
                
                # Hand Action Part
                if not self.arm_only:
                    hand_next_clean = q_next_clean_full[p_start+7 : p_end]
                    # Target = Next + (1-alpha)*Drift
                    hand_target = hand_next_clean + (1.0 - self.correction_alpha) * hand_drift
                    target_action_raw = np.concatenate([action_arm, hand_target])
                else:
                    target_action_raw = action_arm

        # --- 3. Fallback / Validation Logic ---
        # If we didn't drift (validation or probability check), use standard Ground Truth
        if target_action_raw is None:
            # Get raw delta from dataset (handles 'delta' modes natively)
            if self.action_space == 'delta':
                action_gt_full = traj['action_t'][frame_idx]
                target_action_raw = (action_gt_full[:self.num_arm_actions] 
                                     if self.arm_only else action_gt_full)
            
            # Construct absolute target from state t+1
            elif self.action_space == 'absolute':
                q_next_proprio = q_next_clean_full[self.proprio_start_idx : self.proprio_end_idx]
                
                if self.control_mode == 'joint_space':
                    target_action_raw = q_next_proprio[:self.num_arm_actions] if self.arm_only else q_next_proprio
                
                elif self.control_mode == 'task_space':
                    q_next_arm_7d = q_next_proprio[:7]
                    action_gt_arm = self._convert_7d_to_9d(q_next_arm_7d)
                    
                    if self.arm_only:
                        target_action_raw = action_gt_arm
                    else:
                        q_next_hand = q_next_proprio[7:] 
                        target_action_raw = np.concatenate([action_gt_arm, q_next_hand], axis=0)

        # --- 4. Final Normalization ---
        # Ensure target_action_raw is definitely a numpy array of shape (Output_Dim,)
        target_action_raw = np.array(target_action_raw, dtype=np.float32)

        # Normalize State Stack (Apply obs noise here, AFTER drift)
        state_seq_norm = (state_seq_noisy - self.X_mean) / self.X_std
        if self.is_train and self.obs_noise_std > 0:
            state_seq_norm += np.random.normal(0, self.obs_noise_std, state_seq_norm.shape)

        # Normalize Action
        # This was the line causing errors. Now target_action_raw is guaranteed (23,) or (9,) etc.
        action_norm = (target_action_raw - self.y_mean) / self.y_std

        # Flatten if required
        x_out = state_seq_norm.flatten() if self.flatten else state_seq_norm
        
        return torch.from_numpy(x_out).float(), torch.from_numpy(action_norm).float()
    
class JointWeightedMSELoss(nn.Module):
    def __init__(self, action_weights):
        """
        Calculates Mean Squared Error on actions, weighted by joint importance.
        
        With the switch to 9D Ortho6D for absolute task space, we no longer need 
        complex geometric handling (like quaternion double-cover checks) in the loss.
        Standard MSE on the 6D continuous rotation representation is robust and correct.

        Args:
            action_weights (torch.Tensor): Vector of weights, one per dimension.
        """
        super().__init__()
        self.register_buffer('action_weights', action_weights.float())
        self.action_weights = action_weights.float()
        self.mse_loss = nn.MSELoss(reduction='none') 

    def forward(self, pred_norm, target_norm):
        # Standard squared error: (pred - target)^2
        squared_error = self.mse_loss(pred_norm, target_norm) # Shape: (B, D)

        # Apply Weights
        # self.action_weights shape: (D,) -> broadcast to (1, D)
        weighted_squared_error = squared_error * self.action_weights.view(1, -1)

        # Mean over batch and dimensions
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

        # --- Determine Action Dimensions for Slicing ---
        # This logic must match the Dataset class exactly
        if control_mode == 'joint_space':
            arm_action_dim = final_args.num_arm_joints # Usually 7
        elif control_mode == 'task_space':
            if action_space == 'absolute':
                arm_action_dim = 9  # 3D Pos + 6D Rotation (Ortho6D)
            else:
                arm_action_dim = 6  # 3D Lin Vel + 3D Ang Vel (Twist)

        # --- Apply Slicing based on configuration ---
        if final_args.arm_only:
            logging.info(f"ARM-ONLY mode enabled. Slicing ACTIONS to first {arm_action_dim} dimensions.")
            
            for traj in all_trajectories:
                # We ONLY slice the action vector (the output of the policy).
                # The input state still contains full proprioception.
                traj['action_t'] = traj['action_t'][:, :arm_action_dim]

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

        # --- Model setup ---
        # We get dimensions dynamically from the dataset after it has processed the stats
        # train_dataset.y_mean is automatically the correct size (9, 25, 6, 22, etc.)
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

        # --- Weighting Logic ---
        W_HIGH = config.get("joint_high_weight", 2.0)
        W_LOW = config.get("joint_low_weight", 1.0)
        
        # Initialize all weights to High
        action_weights_np = np.full((output_dim,), W_HIGH, dtype=np.float32)
        
        # If we have hand joints, set them to Low
        # We detect this by checking if output > arm_action_dim
        if output_dim > arm_action_dim:
            # The hand joints are the trailing dimensions
            hand_dim = output_dim - arm_action_dim
            action_weights_np[-hand_dim:] = W_LOW
            logging.info(f"Weights: Arm (0-{arm_action_dim-1})={W_HIGH}, Hand ({arm_action_dim}-{output_dim-1})={W_LOW}")
        else:
            logging.info(f"Weights: Arm Only (0-{arm_action_dim-1})={W_HIGH}")

        action_weights_t = torch.from_numpy(action_weights_np).float().to(device)

        # --- Instantiate the Simplified Loss ---
        loss_fn = JointWeightedMSELoss(action_weights=action_weights_t)

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

