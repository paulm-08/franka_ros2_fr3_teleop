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

# --- Logger Setup ---
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

# ===================================================================
# === MODEL DEFINITIONS ===
# ===================================================================
class MLPPolicy(nn.Module):
    def __init__(self, input_dim, output_dim, width=512):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, width),
            nn.ReLU(),
            nn.Dropout(p=0.4), # Increased dropout
            nn.Linear(width, width // 2),
            nn.ReLU(),
            nn.Dropout(p=0.4), # Increased dropout
            nn.Linear(width // 2, output_dim)
        )
    def forward(self, x):
        return self.net(x)

class LSTMPolicy(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_dim=256, num_layers=2):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True, dropout=0.2 if num_layers > 1 else 0)
        self.dropout1 = nn.Dropout(p=0.5) # Aggressive dropout after LSTM
        self.fc1 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.relu = nn.ReLU()
        self.dropout2 = nn.Dropout(p=0.5) # Aggressive dropout after activation
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
    def __init__(self, input_dim, output_dim, num_heads=4, hidden_dim=128, num_layers=2):
        super().__init__()
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=input_dim, nhead=num_heads, dim_feedforward=hidden_dim, batch_first=True
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        self.fc = nn.Linear(input_dim, output_dim)

    def forward(self, x):
        if x.ndim == 2:
            x = x.unsqueeze(1)
        out = self.encoder(x)
        return self.fc(out[:, -1, :])

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
            num_samples = traj['joints_t'].shape[0]
            if num_samples >= self.k:
                for frame_idx in range(self.k - 1, num_samples):
                    self.indices.append((traj_idx, frame_idx))
        
        if norm_stats:
            self.X_mean, self.X_std, self.y_mean, self.y_std = norm_stats
        else:
            # The X_unstacked now includes the goal state
            X_unstacked = np.concatenate([
                np.concatenate([t['tactile_t'], t['visual_t'], t['joints_t'], t['goal_t']], axis=1) 
                for t in trajectories
            ], axis=0)
            y_unstacked = np.concatenate([t['delta_q'] for t in trajectories], axis=0)
            self.X_mean = X_unstacked.mean(axis=0); self.X_std = X_unstacked.std(axis=0)
            self.y_mean = y_unstacked.mean(axis=0); self.y_std = y_unstacked.std(axis=0)
            self.X_std[self.X_std < 1e-9] = 1.0; self.y_std[self.y_std < 1e-9] = 1.0

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, idx):
        traj_idx, frame_idx = self.indices[idx]
        traj = self.trajectories[traj_idx]
        
        start_idx, end_idx = frame_idx - self.k + 1, frame_idx + 1
        
        state_sequence = np.concatenate([
            traj['tactile_t'][start_idx:end_idx],
            traj['visual_t'][start_idx:end_idx],
            traj['joints_t'][start_idx:end_idx],
            traj['goal_t'][start_idx:end_idx]
        ], axis=1)
        
        state_norm = (state_sequence - self.X_mean) / self.X_std
        
        # --- NEW: Add noise ONLY during training ---
        if self.is_train and self.noise_std > 0:
            noise = np.random.normal(0, self.noise_std, state_norm.shape)
            state_norm += noise
            
        action = traj['delta_q'][frame_idx]
        action_norm = (action - self.y_mean) / self.y_std
        
        state_output = state_norm.flatten() if self.flatten else state_norm
            
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
        return LSTMPolicy(input_dim, output_dim)
    elif model_type == "gru":
        return GRUPolicy(input_dim, output_dim)
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
    parser.add_argument("--dataset_pkl", type=str, help="Path to the .pkl dataset file.")
    # FIX: Add the missing output_dir argument with a dynamic default
    parser.add_argument("--output_dir", type=str, default=str(paths.POLICY_MODELS_DIR), help="Directory to save models and logs.")
    parser.add_argument("--model_type", type=str, choices=["mlp", "lstm", "gru"], help="Model architecture.")
    parser.add_argument("--frame_stack", type=int, help="Number of frames to stack (K).")
    parser.add_argument("--width", type=int, help="Width of hidden layers for MLP.")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--batch_size", type=int, default=64)
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--split_ratio", type=float, default=0.85)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--arm_only", action="store_true", help="If set, trains a policy for the arm joints only.")
    parser.add_argument("--num_arm_joints", type=int, default=7, help="The number of joints belonging to the arm.")
    args = parser.parse_args()

    try:
        # --- 1. Interactively build up the configuration ---
        answers = {}
        if not args.dataset_pkl:
            pkl_choices = find_pkl_files(paths.PROCESSED_DATA_DIR)
            if not pkl_choices: logging.error(f"No .pkl files found in {paths.PROCESSED_DATA_DIR}."); return
            answers.update(inquirer.prompt([inquirer.List('dataset_pkl', message="Select the dataset to train on", choices=pkl_choices)]) or {})
        
        if not args.model_type:
            model_questions = [
                inquirer.List('model_type', message="Select model architecture", choices=['mlp', 'lstm', 'gru'], default='mlp'),
                inquirer.Text('frame_stack', message="Enter frame stack size (K)", default='1'),
                inquirer.Text('width', message="Enter MLP width", default='512', ignore=lambda x: x['model_type'] != 'mlp'),
            ]
            answers.update(inquirer.prompt(model_questions) or {})

        # --- 2. Combine args and answers, with args taking precedence ---
        final_args = argparse.Namespace(**vars(args))
        for key, value in answers.items():
            if getattr(final_args, key) is None:
                try: value = int(value)
                except (ValueError, TypeError): pass
                setattr(final_args, key, value)
        
        if not final_args.dataset_pkl:
            logging.info("No dataset selected. Exiting."); return

        # --- 3. Run the Training Logic ---
        device = "cuda" if torch.cuda.is_available() else "cpu"
        logging.info(f"Using device: {device}")
        set_seed(final_args.seed)

        # FIX: Use the final_args object to get the output directory
        output_dir = Path(final_args.output_dir); debug_dir = output_dir / "debug"
        output_dir.mkdir(parents=True, exist_ok=True); debug_dir.mkdir(exist_ok=True)

        with open(paths.WORKSPACE_ROOT / final_args.dataset_pkl, "rb") as f:
            all_trajectories = pickle.load(f)
        
        random.seed(final_args.seed); random.shuffle(all_trajectories)
        split_index = int(len(all_trajectories) * final_args.split_ratio)
        train_trajectories = all_trajectories[:split_index]
        val_trajectories = all_trajectories[split_index:]
        logging.info(f"Loaded {len(all_trajectories)} trajectories: {len(train_trajectories)} train, {len(val_trajectories)} val.")

        if final_args.arm_only:
            logging.info(f"ARM-ONLY mode enabled. Using the first {final_args.num_arm_joints} joints.")
            n = final_args.num_arm_joints
            
            # Slice the joint and action data in each trajectory dictionary
            for traj in all_trajectories: # Apply to all before splitting for consistency
                traj['joints_t'] = traj['joints_t'][:, :n]
                traj['delta_q'] = traj['delta_q'][:, :n]
                # Also slice the goal state's joint component if using GCBC
                if 'goal_t' in traj:
                    goal_joints = traj['goal_t'][:, -23:-23+n] # Assuming goal joints are the last 23 features
                    other_goal_features = traj['goal_t'][:, :-23]
                    traj['goal_t'] = np.concatenate([other_goal_features, goal_joints], axis=1)

        is_sequence_model = final_args.model_type in ["lstm", "gru"]
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

        single_frame_dim = train_dataset.X_mean.shape[0]
        output_dim = train_dataset.y_mean.shape[0]
        input_dim = single_frame_dim * final_args.frame_stack if not is_sequence_model else single_frame_dim

        model = build_model(final_args.model_type, input_dim, output_dim, width=final_args.width).to(device)
        optimizer = optim.Adam(model.parameters(), lr=final_args.lr, weight_decay=1e-4)
        loss_fn = nn.MSELoss()

        logging.info(f"Training with Frame Stacking (K={final_args.frame_stack}) for '{final_args.model_type.upper()}' model.")
        logging.info(f"  Input Dim: {input_dim}, Output Dim: {output_dim}")

        history = {"train_loss": [], "val_loss": []}
        best_val_loss = float('inf')
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

            if avg_val_loss < best_val_loss:
                best_val_loss = avg_val_loss
                torch.save({
                    "state_dict": model.state_dict(),
                    "model_type": final_args.model_type,
                    "input_dim": input_dim, "output_dim": output_dim,
                    "X_mean": train_dataset.X_mean, "X_std": train_dataset.X_std,
                    "y_mean": train_dataset.y_mean, "y_std": train_dataset.y_std,
                    "frame_stack": final_args.frame_stack,
                    "best_val_loss": best_val_loss, "epoch": epoch + 1,
                    "arm_only": final_args.arm_only, "num_arm_joints": final_args.num_arm_joints
                }, model_save_path)
                logging.info(f"  -> New best model saved to {model_save_path} (Val Loss: {best_val_loss:.6f})")
                patience_counter = 0
            else:
                patience_counter += 1

            if patience_counter >= final_args.patience:
                logging.info(f"Validation loss did not improve for {final_args.patience} epochs. Stopping early.")
                break
        
        logging.info(f"🏁 Training complete. Best validation loss: {best_val_loss:.6f}")
        
        plt.figure(figsize=(10, 5))
        plt.plot(history["train_loss"], label="Training Loss")
        plt.plot(history["val_loss"], label="Validation Loss")
        plt.xlabel("Epoch"); plt.ylabel("MSE Loss"); plt.title(f"Training and Validation Loss ({final_args.model_type})")
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

