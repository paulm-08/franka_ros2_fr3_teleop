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

from model_pipeline.dataset_builder import find_config_files
from model_pipeline import paths
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
    
class TransformerPolicy(nn.Module):
    def __init__(self, input_dim, output_dim, num_heads=4, hidden_dim=256, num_layers=2):
        super().__init__()
        # PyTorch's Transformer modules require d_model to be divisible by nhead
        if input_dim % num_heads != 0:
            raise ValueError(f"input_dim ({input_dim}) must be divisible by num_heads ({num_heads}).")
            
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=input_dim, nhead=num_heads, dim_feedforward=hidden_dim, batch_first=True, dropout=0.1
        )
        self.transformer_encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        self.fc = nn.Linear(input_dim, output_dim)

    def forward(self, x):
        # x is expected to be (Batch, SequenceLength, FeatureDim)
        # For Transformer, we often use a special [CLS] token, but for simple BC,
        # taking the output of the last token is a strong baseline.
        transformer_out = self.transformer_encoder(x)
        last_timestep_out = transformer_out[:, -1, :]
        return self.fc(last_timestep_out)

# ===================================================================
# === DATASET CLASS ===
# ===================================================================
class TrajectoryFrameStackDataset(Dataset):
    def __init__(self, trajectories, frame_stack_k, norm_stats=None, flatten=True, is_train=False, noise_std=0.0):
        self.trajectories = trajectories
        self.k = frame_stack_k
        self.flatten = flatten
        # --- NEW: Flags for augmentation ---
        self.is_train = is_train
        self.noise_std = noise_std
        
        self.indices = []
        for traj_idx, traj in enumerate(self.trajectories):
            num_samples = traj['state_t'].shape[0]
            if num_samples >= self.k:
                for frame_idx in range(self.k - 1, num_samples):
                    self.indices.append((traj_idx, frame_idx))
        
        if norm_stats:
            self.X_mean, self.X_std, self.y_mean, self.y_std = norm_stats
        else:
            # The X_unstacked now includes the goal state
            X_unstacked = np.concatenate([
                np.concatenate([t['state_t'], t['goal_t']], axis=1) 
                for t in trajectories
            ], axis=0)
            y_unstacked = np.concatenate([t['action_t'] for t in trajectories], axis=0)
            self.X_mean = X_unstacked.mean(axis=0); self.X_std = X_unstacked.std(axis=0)
            self.y_mean = y_unstacked.mean(axis=0); self.y_std = y_unstacked.std(axis=0)
            self.X_std[self.X_std < 1e-9] = 1.0; self.y_std[self.y_std < 1e-9] = 1.0
        
        self.current_state_dim = trajectories[0]['state_t'].shape[1]

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        traj_idx, frame_idx = self.indices[idx]
        traj = self.trajectories[traj_idx]
        
        start_idx, end_idx = frame_idx - self.k + 1, frame_idx + 1
        
        # --- 1. Build the full state sequence (current + goal) ---
        full_state_sequence = np.concatenate([
            traj['state_t'][start_idx:end_idx],
            traj['goal_t'][start_idx:end_idx],
        ], axis=1)
        
        # --- 2. Normalize the entire sequence ---
        full_state_norm = (full_state_sequence - self.X_mean) / self.X_std
        
        # --- 3. Split state and goal BEFORE adding noise ---
        current_state_norm = full_state_norm[:, :self.current_state_dim]
        goal_state_norm = full_state_norm[:, self.current_state_dim:]
        
        # --- 4. Add noise ONLY to the current state during training ---
        if self.is_train and self.noise_std > 0:
            noise = np.random.normal(0, self.noise_std, current_state_norm.shape).astype(np.float32)
            current_state_norm += noise
        
        # --- 5. Recombine the (potentially noisy) state with the CLEAN goal ---
        final_state_norm = np.concatenate([current_state_norm, goal_state_norm], axis=1)
        
        action = traj['action_t'][frame_idx]
        action_norm = (action - self.y_mean) / self.y_std
        
        state_output = final_state_norm.flatten() if self.flatten else final_state_norm
            
        return torch.from_numpy(state_output).float(), torch.from_numpy(action_norm).float()
    
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

def find_pkl_files(search_path):
    logging.info(f"Searching for processed datasets (.pkl) in: {search_path}...")
    all_files = [p.relative_to(paths.WORKSPACE_ROOT) for p in search_path.glob("*.pkl")]
    cleaned_files = sorted([p for p in all_files if 'cleaned' in str(p)])
    other_files = sorted([p for p in all_files if 'cleaned' not in str(p)])
    return [str(p) for p in cleaned_files + other_files]

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
                flatten=(not is_sequence_model),
                is_train=True,         # <-- Enable training mode
                noise_std=0.01         # <-- Add a small amount of noise
            )
            
            # Validation dataset should NOT have noise
            val_dataset = TrajectoryFrameStackDataset(
                val_trajectories, 
                final_args.frame_stack,
                norm_stats=(train_dataset.X_mean, train_dataset.X_std, train_dataset.y_mean, train_dataset.y_std),
                flatten=(not is_sequence_model),
                is_train=False         # <-- Keep this False
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
                flatten=(not is_sequence_model),
                is_train=True,         # <-- Enable training mode
                noise_std=0.01         # <-- Add a small amount of noise
            )
            
            if len(train_dataset) == 0:
                logging.error("Training dataset is empty!"); return

            train_loader = DataLoader(train_dataset, batch_size=final_args.batch_size, shuffle=True, num_workers=4)

        single_frame_dim = train_dataset.X_mean.shape[0]
        output_dim = train_dataset.y_mean.shape[0]
        input_dim = single_frame_dim * final_args.frame_stack if not is_sequence_model else single_frame_dim

        # --- Build Model ---
        model = build_model(
            final_args.model_type, input_dim, output_dim, 
            width=final_args.width,
            num_heads=final_args.num_heads,
            hidden_dim=final_args.hidden_dim,
            num_layers=final_args.num_layers
        ).to(device)
        optimizer = optim.Adam(model.parameters(), lr=final_args.lr, weight_decay=1e-3)
        loss_fn = nn.MSELoss()

        logging.info(f"Training with Frame Stacking (K={final_args.frame_stack}) for '{final_args.model_type.upper()}' model.")
        logging.info(f"  Input Dim: {input_dim}, Output Dim: {output_dim}")

        history = {"train_loss": [], "val_loss": []}
        best_val_loss = float('inf')
        best_train_loss = float('inf')
        patience_counter = 0
        model_save_path = output_dir / f"policy_{final_args.model_type}_best.pt"

        for epoch in range(final_args.epochs):
            model.train()
            total_train_loss = 0.0
            for state_norm, action_norm in train_loader:
                state_norm, action_norm = state_norm.to(device), action_norm.to(device)
                pred_norm = model(state_norm)
                loss = loss_fn(pred_norm, action_norm)
                optimizer.zero_grad(); loss.backward(); optimizer.step()
                total_train_loss += loss.item()
            avg_train_loss = total_train_loss / len(train_loader)
            history["train_loss"].append(avg_train_loss)

            if final_args.validation:
                model.eval()
                total_val_loss = 0.0
                with torch.no_grad():
                    for state_norm, action_norm in val_loader:
                        state_norm, action_norm = state_norm.to(device), action_norm.to(device)
                        pred_norm = model(state_norm)
                        loss = loss_fn(pred_norm, action_norm)
                        total_val_loss += loss.item()
                avg_val_loss = total_val_loss / len(val_loader)
                history["val_loss"].append(avg_val_loss)

                logging.info(f"Epoch {epoch+1:03d}/{final_args.epochs} | Train Loss: {avg_train_loss:.6f} | Val Loss: {avg_val_loss:.6f}")
            else:
                logging.info(f"Epoch {epoch+1:03d}/{final_args.epochs} | Train Loss: {avg_train_loss:.6f}")

                
            if (final_args.validation and avg_val_loss < best_val_loss) or (not final_args.validation and avg_train_loss<best_train_loss):
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
                    "arm_only": final_args.arm_only, "num_arm_joints": final_args.num_arm_joints
                }, model_save_path)

                if final_args.validation:
                    logging.info(f"  -> New best model saved to {model_save_path} (Val Loss: {best_val_loss:.6f})")
                else:
                    logging.info(f"  -> New best model saved to {model_save_path} (Train Loss: {best_train_loss:.6f})")

                patience_counter = 0
            else:
                patience_counter += 1

            if patience_counter >= final_args.patience:
                logging.info(f"Validation loss did not improve for {final_args.patience} epochs. Stopping early.")
                break
        
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
        plot_save_path = debug_dir / f"loss_curve_{final_args.model_type}.png"
        plt.savefig(plot_save_path)
        logging.info(f"📉 Saved loss curve to {plot_save_path}")

    except (KeyboardInterrupt, TypeError) as e:
        logging.error(f"An error occurred: {e}", exc_info=True) # Log the full traceback
        logging.info("\nTraining configuration cancelled or failed.")
        return

if __name__ == "__main__":
    main()

