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

from model_pipeline.utils import find_config_files
from model_pipeline import paths
from model_pipeline.utils import find_pkl_files

# --- Logger Setup ---
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

# ===================================================================
# === MODEL DEFINITIONS ===
# ===================================================================
class MLPPolicy(nn.Module):
    """A dynamically generated MLP with a configurable number of layers and width."""
    def __init__(self, input_dim, output_dim, width=512, num_layers=2, dropout_p=0.4):
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
    def __init__(self, input_dim, output_dim, hidden_dim=256, num_layers=2):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True, dropout=0.2 if num_layers > 1 else 0)
        self.dropout1 = nn.Dropout(p=0.5)
        self.fc1 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.relu = nn.ReLU()
        self.dropout2 = nn.Dropout(p=0.5)
        self.fc2 = nn.Linear(hidden_dim // 2, output_dim)
    def forward(self, x):
        lstm_out, _ = self.lstm(x)
        last_timestep_out = lstm_out[:, -1, :]
        x = self.dropout1(last_timestep_out)
        x = self.relu(self.fc1(x))
        x = self.dropout2(x)
        return self.fc2(x)
    
class GRUPolicy(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dim=256, num_layers=2):
        super().__init__()
        self.gru = nn.GRU(input_dim, hidden_dim, num_layers, batch_first=True, dropout=0.2 if num_layers > 1 else 0)
        self.fc1 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden_dim // 2, output_dim)
    def forward(self, x):
        gru_out, _ = self.gru(x)
        last_timestep_out = gru_out[:, -1, :]
        x = self.relu(self.fc1(last_timestep_out))
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
                 obs_noise_std=0.0, drift_noise_std=0.0, return_joints=False):
        """
        obs_noise_std  : small noise for robustness to observations (all features)
        drift_noise_std: larger noise for training drift recovery (only q_t)
        """
        self.trajectories = trajectories
        self.k = frame_stack_k
        self.flatten = flatten
        self.is_train = is_train
        self.obs_noise_std = obs_noise_std
        self.drift_noise_std = drift_noise_std
        self.return_joints = return_joints

        # --- Config options ---
        self.arm_only = arm_only
        self.control_mode = config.get('control_mode', 'joint_space')
        self.use_goal = config.get('use_goal', False)
        self.num_arm_actions = 6 if self.control_mode == 'task_space' else 7

        self.indices = []
        for traj_idx, traj in enumerate(self.trajectories):
            num_samples = traj['state_t'].shape[0]
            # -1 to leave one sample for q_next
            if num_samples >= self.k + 1: 
                for frame_idx in range(self.k - 1, num_samples - 1):
                    self.indices.append((traj_idx, frame_idx))
        
        # --- Normalization stats ---
        if norm_stats:
            self.X_mean, self.X_std, self.y_mean, self.y_std = norm_stats
        else:
            if self.use_goal:
                X_unstacked = np.concatenate([
                    np.concatenate([t['state_t'], t['goal_t']], axis=1)
                    for t in trajectories
                ], axis=0)
            else:
                X_unstacked = np.concatenate([t['state_t'] for t in trajectories], axis=0)

            y_unstacked_full = np.concatenate([t['action_t'] for t in trajectories], axis=0)
            y_unstacked = (y_unstacked_full[:, :self.num_arm_actions]
                           if self.arm_only else y_unstacked_full)

            self.X_mean = X_unstacked.mean(axis=0)
            self.X_std  = X_unstacked.std(axis=0)
            self.y_mean = y_unstacked.mean(axis=0)
            self.y_std  = y_unstacked.std(axis=0)
            self.X_std[self.X_std < 1e-9] = 1.0
            self.y_std[self.y_std < 1e-9] = 1.0

        self.current_state_dim = trajectories[0]['state_t'].shape[1]
        
        # --- FIX: Get the mean/std for ONLY the proprioceptive state ---
        # This is needed to normalize q_t_noisy
        TACTILE_DIM = 24
        ARM_PROP_DIM = 7 # 7D for pose or 7D for joints
        self.proprio_start_idx = TACTILE_DIM
        self.proprio_end_idx = self.proprio_start_idx + ARM_PROP_DIM

        # --- Extract the mean/std for ONLY the arm proprioceptive state ---
        self.proprio_mean = self.X_mean[self.proprio_start_idx:self.proprio_end_idx]
        self.proprio_std = self.X_std[self.proprio_start_idx:self.proprio_end_idx]

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        traj_idx, frame_idx = self.indices[idx]
        traj = self.trajectories[traj_idx]
        start_idx, end_idx = frame_idx - self.k + 1, frame_idx + 1

        # --- Build stacked states ---
        if self.use_goal:
            full_state_sequence = np.concatenate([
                traj['state_t'][start_idx:end_idx],
                traj['goal_t'][start_idx:end_idx],
            ], axis=1)
        else:
            full_state_sequence = traj['state_t'][start_idx:end_idx]

        full_state_norm = (full_state_sequence - self.X_mean) / self.X_std
        current_state_norm = full_state_norm[:, :self.current_state_dim]
        goal_state_norm = full_state_norm[:, self.current_state_dim:] if self.use_goal else None

        # --- 1. Create the state for IMITATION loss (obs_noise only) ---
        current_state_obs_noise = current_state_norm.copy()
        if self.is_train and self.obs_noise_std > 0:
            current_state_obs_noise += np.random.normal(0, self.obs_noise_std, current_state_norm.shape)

        if self.use_goal:
            final_state_obs_noise = np.concatenate([current_state_obs_noise, goal_state_norm], axis=1)
        else:
            final_state_obs_noise = current_state_obs_noise
            
        state_output_bc = final_state_obs_noise.flatten() if self.flatten else final_state_obs_noise

        # --- 2. Get the standard action (for IMITATION loss) ---
        action = traj['action_t'][frame_idx]
        if self.arm_only:
            action = action[:self.num_arm_actions]
        action_norm = (action - self.y_mean) / self.y_std

        if not self.return_joints:
            return torch.from_numpy(state_output_bc).float(), torch.from_numpy(action_norm).float()

        # --- 3. Create the state for DRIFT loss ---
        q_t_clean = traj['state_t'][frame_idx, self.proprio_start_idx:self.proprio_end_idx].astype(np.float32)
        q_next_gt = traj['state_t'][frame_idx + 1, self.proprio_start_idx:self.proprio_end_idx].astype(np.float32)

        if self.is_train and self.drift_noise_std > 0:
            q_t_noisy = q_t_clean + np.random.normal(0, self.drift_noise_std, q_t_clean.shape).astype(np.float32)
        else:
            q_t_noisy = q_t_clean # Use clean state for validation drift loss

        # Normalize the noisy state
        q_t_noisy_norm = (q_t_noisy - self.proprio_mean) / self.proprio_std
        
        # Create the drifted state vector (uses obs_noise on all features)
        final_state_norm_drifted = final_state_obs_noise.copy()
        
        # --- Inject the large drift noise ---
        # We replace the arm proprio features in the *last* frame of the stack
        final_state_norm_drifted[-1, self.proprio_start_idx:self.proprio_end_idx] = q_t_noisy_norm
        
        state_output_drift = final_state_norm_drifted.flatten() if self.flatten else final_state_norm_drifted

        return (
            torch.from_numpy(state_output_bc).float(),    # State for BC loss
            torch.from_numpy(state_output_drift).float(), # State for Drift loss
            torch.from_numpy(action_norm).float(),       # Target for BC loss
            torch.from_numpy(q_t_noisy).float(),         # Un-normalized q_t
            torch.from_numpy(q_next_gt).float()          # Target for Drift loss
        )
        
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
        action_representation = config.get("action_representation", "delta")
        logging.info(f"Action representation: {action_representation}")
        use_goal = config.get("use_goal", False)
        logging.info(f"Use goal conditioning: {use_goal}")
        use_drift_regularizer = config.get("use_drift_regularizer", False)
        logging.info(f"Use drift regularizer: {use_drift_regularizer}")
        drift_loss_weight = config.get("drift_loss_weight", 0.1)
        if use_drift_regularizer:
            logging.info(f"     Drift loss weight: {drift_loss_weight}")
        joint_indices = config.get("joint_indices", [24, 31])
        obs_noise_std_train = config.get("obs_noise_std_train", 0.01)
        obs_noise_std_val = config.get("obs_noise_std_val", 0.0)
        drift_noise_std = config.get("drift_noise_std", 0.05)

        logging.info(f"Observation noise std (train): {obs_noise_std_train}")
        if final_args.validation:
            logging.info(f"Observation noise std (val): {obs_noise_std_val}")
        logging.info(f"Drift noise std: {drift_noise_std}")

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
                return_joints=use_drift_regularizer
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
                return_joints=use_drift_regularizer
            )

            if len(train_dataset) == 0:
                logging.error("Training dataset is empty!"); return

            train_loader = DataLoader(train_dataset, batch_size=final_args.batch_size, shuffle=True, num_workers=4)
            val_loader = DataLoader(val_dataset, batch_size=final_args.batch_size * 2, num_workers=4)

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
                return_joints=use_drift_regularizer
            )

            if len(train_dataset) == 0:
                logging.error("Training dataset is empty!"); return

            train_loader = DataLoader(train_dataset, batch_size=final_args.batch_size, shuffle=True, num_workers=4)

        # --- Model setup ---
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
        loss_fn = nn.MSELoss()

        y_mean_t = torch.from_numpy(train_dataset.y_mean).float().to(device)
        y_std_t  = torch.from_numpy(train_dataset.y_std).float().to(device)

        logging.info(f"Training with Frame Stacking (K={final_args.frame_stack}) for '{final_args.model_type.upper()}' model.")
        logging.info(f"  Input Dim: {input_dim}, Output Dim: {output_dim}")

        history = {"train_loss": [], "val_loss": []}
        best_val_loss = float('inf')
        best_train_loss = float('inf')
        patience_counter = 0
        model_save_path = output_dir / f"policy_{final_args.model_type}_{control_mode}.pt"

        # --- Training loop ---
        try:
            for epoch in range(final_args.epochs):
                model.train()
                total_train_loss = 0.0
                total_imitation_loss = 0.0
                total_drift_loss = 0.0

                for batch in train_loader:
                    if use_drift_regularizer and control_mode == 'joint_space':
                        state_bc_norm, state_drift_norm, action_norm, q_t_noisy, q_next_gt = batch
                        state_drift_norm = state_drift_norm.to(device)
                        q_t_noisy, q_next_gt = q_t_noisy.to(device), q_next_gt.to(device)
                    else:
                        state_bc_norm, action_norm = batch
                        use_drift_regularizer = False

                    state_bc_norm, action_norm = state_bc_norm.to(device), action_norm.to(device)
                    optimizer.zero_grad()

                    # --- 1. Imitation Loss (BC Loss) ---
                    # Use the state with only observation noise
                    pred_norm_bc = model(state_bc_norm)
                    imitation_loss = loss_fn(pred_norm_bc, action_norm)

                    if use_drift_regularizer:
                        # --- 2. Drift Loss ---
                        # Use the state with the large drift noise
                        pred_norm_drift = model(state_drift_norm)
                        
                        # Denormalize using the Tensors
                        delta_q_pred = (pred_norm_drift * y_std_t) + y_mean_t
                        
                        # Predict the next state in UN-NORMALIZED space
                        q_pred_next = q_t_noisy + delta_q_pred
                        
                        # Compare two UN-NORMALIZED states
                        drift_loss = loss_fn(q_pred_next, q_next_gt)
                        
                        # --- 3. Combine the losses ---
                        loss = (1 - drift_loss_weight) * imitation_loss + (drift_loss_weight * drift_loss)
                        
                        total_imitation_loss += imitation_loss.item()
                        total_drift_loss += drift_loss.item()
                    else:
                        loss = imitation_loss # No drift loss

                    loss.backward()
                    optimizer.step()
                    total_train_loss += loss.item()

                avg_train_loss = total_train_loss / len(train_loader)
                avg_imitation_loss = total_imitation_loss / len(train_loader) if use_drift_regularizer else avg_train_loss
                avg_drift_loss = total_drift_loss / len(train_loader)
                history["train_loss"].append(avg_train_loss)

                # --- Validation phase ---
                if final_args.validation:
                    model.eval()
                    total_val_loss = 0.0
                    total_val_imitation_loss = 0.0
                    total_val_drift_loss = 0.0
                    
                    with torch.no_grad():
                        for batch in val_loader:
                            if use_drift_regularizer:
                                state_bc_norm, state_drift_norm, action_norm, q_t_noisy, q_next_gt = batch
                                state_drift_norm = state_drift_norm.to(device)
                                q_t_noisy, q_next_gt = q_t_noisy.to(device), q_next_gt.to(device)
                            else:
                                state_bc_norm, action_norm = batch
                                use_drift_regularizer = False

                            # --- 1. Imitation Loss ---
                            pred_norm_bc = model(state_bc_norm)
                            imitation_loss = loss_fn(pred_norm_bc, action_norm)

                            if use_drift_regularizer:
                                # --- 2. Drift Loss ---
                                pred_norm_drift = model(state_drift_norm)
                                delta_q_pred = (pred_norm_drift * y_std_t) + y_mean_t
                                q_pred_next = q_t_noisy + delta_q_pred # q_t_noisy is clean in validation
                                drift_loss = loss_fn(q_pred_next, q_next_gt)
                                
                                loss = (1 - drift_loss_weight) * imitation_loss + (drift_loss_weight * drift_loss)
                                
                                total_val_imitation_loss += imitation_loss.item()
                                total_val_drift_loss += drift_loss.item()
                            else:
                                loss = imitation_loss
                                
                            total_val_loss += loss.item()

                    avg_val_loss = total_val_loss / len(val_loader)
                    avg_val_imitation = total_val_imitation_loss / len(val_loader) if use_drift_regularizer else avg_val_loss
                    avg_val_drift = total_val_drift_loss / len(val_loader)
                    history["val_loss"].append(avg_val_loss)
                    
                    # --- NEW: Updated logging ---
                    log_msg = f"Epoch {epoch+1:03d}/{final_args.epochs} | Train: {avg_train_loss:.6f} | Val: {avg_val_loss:.6f}"
                    if use_drift_regularizer:
                        log_msg += f" | (Train BC: {avg_imitation_loss:.6f}, Drift: {avg_drift_loss:.6f})"
                        log_msg += f" | (Val BC: {avg_val_imitation:.6f}, Drift: {avg_val_drift:.6f})"
                    logging.info(log_msg)
                    
                else:
                    log_msg = f"Epoch {epoch+1:03d}/{final_args.epochs} | Train Loss: {avg_train_loss:.6f}"
                    if use_drift_regularizer:
                        log_msg += f" | (Train BC: {avg_imitation_loss:.6f}, Drift: {avg_drift_loss:.6f})"
                    logging.info(log_msg)


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

