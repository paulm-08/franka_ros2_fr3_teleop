#!/usr/bin/env python3
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import os, logging, random, argparse, pickle
from pathlib import Path
import matplotlib.pyplot as plt

# --- Logger Setup ---
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()]
)

# ===================================================================
# === 1. MODEL DEFINITIONS ===
# ===================================================================
class MLPPolicy(nn.Module):
    def __init__(self, input_dim, output_dim, width=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, width),
            nn.ReLU(),
            nn.Dropout(p=0.2),
            nn.Linear(width, width // 2),
            nn.ReLU(),
            nn.Dropout(p=0.2),
            nn.Linear(width // 2, output_dim)
        )
    def forward(self, x):
        return self.net(x)

class LSTMPolicy(nn.Module):
    """A more robust LSTM Policy for sequence data."""
    def __init__(self, input_dim, output_dim, hidden_dim=128, num_layers=1):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True, dropout=0.2 if num_layers > 1 else 0)
        self.fc1 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.relu = nn.ReLU()
        self.fc2 = nn.Linear(hidden_dim // 2, output_dim)

    def forward(self, x):
        # x has shape (Batch, K, Feature_Dim)
        lstm_out, _ = self.lstm(x)
        # We only care about the output from the last timestep
        last_timestep_out = lstm_out[:, -1, :]
        x = self.relu(self.fc1(last_timestep_out))
        return self.fc2(x)

class GRUPolicy(nn.Module):
    """A more robust GRU Policy for sequence data."""
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
# === 2. DATASET CLASS ===
# ===================================================================
class TrajectoryFrameStackDataset(Dataset):
    def __init__(self, trajectories, frame_stack_k, norm_stats=None, flatten=True):
        self.trajectories = trajectories
        self.k = frame_stack_k
        self.flatten = flatten
        self.indices = []
        self.is_training = (norm_stats is None)
        
        for traj_idx, traj in enumerate(self.trajectories):
            num_samples = traj['joints_t'].shape[0]
            if num_samples >= self.k:
                for frame_idx in range(self.k - 1, num_samples):
                    self.indices.append((traj_idx, frame_idx))
        
        if norm_stats:
            self.X_mean, self.X_std, self.y_mean, self.y_std = norm_stats
        else:
            X_unstacked = np.concatenate([np.concatenate([t['tactile_t'], t['visual_t'], t['joints_t']], axis=1) for t in trajectories], axis=0)
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
            traj['joints_t'][start_idx:end_idx]
        ], axis=1)
        state_norm = (state_sequence - self.X_mean) / self.X_std
        
        action = traj['delta_q'][frame_idx]
        action_norm = (action - self.y_mean) / self.y_std
        
        state_output = state_norm.flatten() if self.flatten else state_norm

        state_tensor = torch.from_numpy(state_output).float()
        action_tensor = torch.from_numpy(action_norm).float()

        # --- ADD AUGMENTATION ---
        if self.is_training:
            # Add a small amount of Gaussian noise
            state_tensor += torch.randn_like(state_tensor) * 0.02

        return state_tensor, action_tensor

    def log_stats(self):
        """NEW: Helper to log normalization statistics."""
        logging.info("--- Normalization Stats ---")
        logging.info(f"  State (X) mean range: [{self.X_mean.min():.4f}, {self.X_mean.max():.4f}]")
        logging.info(f"  State (X) std range:  [{self.X_std.min():.4f}, {self.X_std.max():.4f}]")
        logging.info(f"  Action (y) mean range: [{self.y_mean.min():.4f}, {self.y_mean.max():.4f}]")
        logging.info(f"  Action (y) std range:  [{self.y_std.min():.4f}, {self.y_std.max():.4f}]")
        logging.info("---------------------------")

# ===================================================================
# === 3. UTILITY FUNCTIONS ===
# ===================================================================
def set_seed(seed=42):
    np.random.seed(seed); random.seed(seed); torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False

def build_model(model_type, input_dim, output_dim, **kwargs):
    """Factory function to build models with extra arguments."""
    if model_type == "mlp":
        return MLPPolicy(input_dim, output_dim, width=kwargs.get("width", 256))
    elif model_type == "lstm":
        return LSTMPolicy(input_dim, output_dim, hidden_dim=kwargs.get("width", 128)) # Can add hidden_dim, etc. here later
    elif model_type == "gru":
        return GRUPolicy(input_dim, output_dim)
    elif model_type == "transformer":
        return TransformerPolicy(input_dim, output_dim)
    else:
        raise ValueError(f"Unknown model_type '{model_type}'")
    
# ===================================================================
# === 4. MAIN TRAINING SCRIPT ===
# ===================================================================
def main():
    parser = argparse.ArgumentParser(description="Train a policy with frame stacking.")
    parser.add_argument("--dataset_pkl", type=str, required=True, help="Path to the .pkl dataset file.")
    parser.add_argument("--frame_stack", type=int, default=1, help="Number of frames to stack (K).")
    parser.add_argument("--output_dir", type=str, default="data/models", help="Directory to save models.")
    parser.add_argument("--epochs", type=int, default=200, help="Number of training epochs.")
    parser.add_argument("--batch_size", type=int, default=64, help="Batch size for training.")
    parser.add_argument("--lr", type=float, default=1e-4, help="Learning rate.")
    parser.add_argument("--model_type", type=str, default="mlp", choices=["mlp", "lstm", "gru"], help="Model architecture.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility.")
    parser.add_argument("--patience", type=int, default=20, help="Patience for early stopping.")
    parser.add_argument("--split_ratio", type=float, default=0.85, help="Train/val split ratio.")
    parser.add_argument("--width", type=int, default=256, help="Width of hidden layers for MLP.")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    logging.info(f"Using device: {device}")
    set_seed(args.seed)

    output_dir = Path(args.output_dir); debug_dir = output_dir / "debug"
    output_dir.mkdir(parents=True, exist_ok=True); debug_dir.mkdir(exist_ok=True)

    # --- 1. Load Data and Split Trajectories ---
    with open(args.dataset_pkl, "rb") as f: all_trajectories = pickle.load(f)
    random.seed(args.seed); random.shuffle(all_trajectories)
    split_index = int(len(all_trajectories) * args.split_ratio)
    train_trajectories = all_trajectories[:split_index]
    val_trajectories = all_trajectories[split_index:]
    logging.info(f"Loaded {len(all_trajectories)} trajectories: {len(train_trajectories)} train, {len(val_trajectories)} val.")

    # --- 2. Create Datasets and Dataloaders ---
    is_sequence_model = args.model_type in ["lstm", "gru", "transformer"]
    
    train_dataset = TrajectoryFrameStackDataset(train_trajectories, args.frame_stack, flatten=(not is_sequence_model))
    train_dataset.log_stats() # NEW: Log the calculated stats for transparency
    
    val_dataset = TrajectoryFrameStackDataset(val_trajectories, args.frame_stack,
                                              norm_stats=(train_dataset.X_mean, train_dataset.X_std,
                                                          train_dataset.y_mean, train_dataset.y_std),
                                              flatten=(not is_sequence_model))

    if len(train_dataset) == 0:
        logging.error("Training dataset is empty! Check trajectory lengths and frame_stack size."); return

    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size * 2, num_workers=4)

    # --- 3. Determine Model Dimensions ---
    single_frame_dim = train_dataset.X_mean.shape[0]
    output_dim = train_dataset.y_mean.shape[0]
    input_dim = single_frame_dim * args.frame_stack if not is_sequence_model else single_frame_dim
    
    # --- 4. Build Model and Log Sanity Checks ---
    model = build_model(args.model_type, input_dim, output_dim, width=args.width).to(device)
    optimizer = optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-4)
    # loss_fn = nn.MSELoss()
    loss_fn = nn.SmoothL1Loss()  # More robust to outliers than MSE

    logging.info(f"Training with Frame Stacking (K={args.frame_stack}) for '{args.model_type.upper()}' model.")
    logging.info(f"  Input Dim: {input_dim}, Output Dim: {output_dim}")
    logging.info(f"  Total trainable parameters: {sum(p.numel() for p in model.parameters() if p.requires_grad):,}")

    # NEW: Sanity check one batch from the dataloader
    try:
        sample_state, sample_action = next(iter(train_loader))
        logging.info(f"  Sanity Check - Batch State Shape: {list(sample_state.shape)}")
        logging.info(f"  Sanity Check - Batch Action Shape: {list(sample_action.shape)}")
        logging.info(f"  Sanity Check - Normalized State Mean: {sample_state.mean():.4f}, Std: {sample_state.std():.4f}")
        logging.info(f"  Sanity Check - Normalized Action Mean: {sample_action.mean():.4f}, Std: {sample_action.std():.4f}")
        assert abs(sample_state.mean()) < 0.1 and abs(sample_state.std() - 1.0) < 0.2, "Normalization seems off!"
    except Exception as e:
        logging.error(f"Sanity check failed: {e}")
        return

    # --- 5. Training & Validation Loop ---
    history = {"train_loss": [], "val_loss": []}
    best_val_loss = float('inf')
    patience_counter = 0
    model_save_path = output_dir / f"policy_{args.model_type}_best.pt"

    for epoch in range(args.epochs):
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

        logging.info(f"Epoch {epoch+1:03d}/{args.epochs} | Train Loss: {avg_train_loss:.6f} | Val Loss: {avg_val_loss:.6f}")

        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            torch.save({
                "state_dict": model.state_dict(),
                "model_type": args.model_type,
                "input_dim": input_dim, "output_dim": output_dim,
                "X_mean": train_dataset.X_mean, "X_std": train_dataset.X_std,
                "y_mean": train_dataset.y_mean, "y_std": train_dataset.y_std,
                "frame_stack": args.frame_stack,
                "best_val_loss": best_val_loss, "epoch": epoch + 1,
            }, model_save_path)
            logging.info(f"  -> New best model saved to {model_save_path} (Val Loss: {best_val_loss:.6f})")
            patience_counter = 0
        else:
            patience_counter += 1

        if patience_counter >= args.patience:
            logging.info(f"Validation loss did not improve for {args.patience} epochs. Stopping early.")
            break
    
    logging.info(f"🏁 Training complete. Best validation loss: {best_val_loss:.6f}")
    
    # --- Plotting ---
    plt.figure(figsize=(10, 5))
    plt.plot(history["train_loss"], label="Training Loss")
    plt.plot(history["val_loss"], label="Validation Loss")
    plt.xlabel("Epoch"); plt.ylabel("MSE Loss"); plt.title(f"Training and Validation Loss ({args.model_type})")
    plt.legend(); plt.grid(True)
    plot_save_path = debug_dir / f"loss_curve_{args.model_type}.png"
    plt.savefig(plot_save_path)
    logging.info(f"📉 Saved loss curve to {plot_save_path}")

if __name__ == "__main__":
    main()