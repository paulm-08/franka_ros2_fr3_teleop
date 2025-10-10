#!/usr/bin/env python3
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import os, logging, random, argparse, matplotlib.pyplot as plt
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# === MODEL DEFINITIONS ===
class MLPPolicy(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 512), # Increased from 128
            nn.ReLU(),
            nn.Dropout(p=0.6),       # Using a lower dropout
            nn.Linear(512, 512),     # Increased from 128
            nn.ReLU(),
            nn.Dropout(p=0.6),       # Using a lower dropout
            nn.Linear(512, output_dim)
        )
    def forward(self, x):
        return self.net(x)
    
class LSTMPolicy(nn.Module):
    """Simple LSTM (for pre-stacked inputs, T=frames already flattened)"""
    def __init__(self, input_dim, output_dim, hidden_size=128, num_layers=1):
        super().__init__()
        self.lstm = nn.LSTM(input_dim, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_dim)

    def forward(self, x):
        # If inputs are pre-stacked (flattened), just treat as single timestep
        # Otherwise expect (B, T, F)
        if x.ndim == 2:
            x = x.unsqueeze(1)
        out, _ = self.lstm(x)
        return self.fc(out[:, -1, :])

class GRUPolicy(nn.Module):
    def __init__(self, input_dim, output_dim, hidden_size=128, num_layers=1):
        super().__init__()
        self.gru = nn.GRU(input_dim, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_dim)

    def forward(self, x):
        if x.ndim == 2:
            x = x.unsqueeze(1)
        out, _ = self.gru(x)
        return self.fc(out[:, -1, :])

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

# === UTILS ===
def set_seed(seed=42):
    np.random.seed(seed)
    random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False

def build_model(model_type, input_dim, output_dim):
    if model_type == "mlp":
        return MLPPolicy(input_dim, output_dim)
    elif model_type == "lstm":
        return LSTMPolicy(input_dim, output_dim)
    elif model_type == "gru":
        return GRUPolicy(input_dim, output_dim)
    elif model_type == "transformer":
        return TransformerPolicy(input_dim, output_dim)
    else:
        raise ValueError(f"Unknown model_type '{model_type}'")

# === MAIN TRAINING ===
def main():
    parser = argparse.ArgumentParser(description="Train a policy for robot control.")
    parser.add_argument("--input_file", type=str, default="data/processed/dataset_split.npz", help="Path to the split dataset file.")
    parser.add_argument("--output_dir", type=str, default="data/models", help="Directory to save models and logs.")
    parser.add_argument("--epochs", type=int, default=100, help="Number of training epochs.")
    parser.add_argument("--batch_size", type=int, default=64, help="Batch size for training.")
    parser.add_argument("--lr", type=float, default=1e-3, help="Learning rate.")
    parser.add_argument("--model_type", type=str, default="mlp", choices=["mlp", "lstm", "gru", "transformer"], help="The model architecture to use.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility.")
    parser.add_argument("--patience", type=int, default=20, help="Patience for early stopping.")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    logging.info(f"Using device: {device}")
    set_seed(args.seed)

    # Create output directories
    output_dir = Path(args.output_dir)
    debug_dir = output_dir / "debug"
    output_dir.mkdir(parents=True, exist_ok=True)
    debug_dir.mkdir(exist_ok=True)

    # --- Load Data ---
    data = np.load(args.input_file)
    X_train_np, y_train_np = data["X_train"], data["y_train"]
    X_val_np, y_val_np = data["X_val"], data["y_val"]

    # --- Normalization (Robust Version) ---
    X_mean = X_train_np.mean(axis=0, keepdims=True)
    X_std = X_train_np.std(axis=0, keepdims=True)

    # FIX: Identify constant features (where std is near zero)
    zero_std_indices = np.where(X_std < 1e-9)[1]
    if len(zero_std_indices) > 0:
        logging.warning(f"Found {len(zero_std_indices)} constant features at indices: {zero_std_indices}")
        # Replace the std of these features with 1.0 to avoid division by zero.
        # This means constant features will be centered but not scaled.
        X_std[0, zero_std_indices] = 1.0

    y_mean, y_std = y_train_np.mean(axis=0), y_train_np.std(axis=0)
    # Also make the action normalization robust
    y_std[y_std < 1e-9] = 1.0

    X_train = torch.tensor((X_train_np - X_mean) / X_std, dtype=torch.float32)
    X_val   = torch.tensor((X_val_np - X_mean) / X_std, dtype=torch.float32)
    y_train = torch.tensor((y_train_np - y_mean) / y_std, dtype=torch.float32)
    y_val   = torch.tensor((y_val_np - y_mean) / y_std, dtype=torch.float32)

    train_loader = DataLoader(TensorDataset(X_train, y_train), batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(TensorDataset(X_val, y_val), batch_size=args.batch_size * 2) # Larger batch size for validation is fine

    # --- Model ---
    model = build_model(args.model_type, X_train.shape[1], y_train.shape[1]).to(device)
    # Add the weight_decay parameter
    optimizer = optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-3)
    loss_fn = nn.MSELoss()

    logging.info(f"Training Model: {args.model_type.upper()} | Input Dim: {X_train.shape[1]}, Output Dim: {y_train.shape[1]}")
    
    # --- Training & Validation Loop ---
    history = {"train_loss": [], "val_loss": []}
    best_val_loss = float('inf')
    patience = args.patience # Stop after 10 epochs with no improvement
    patience_counter = 0
    model_save_path = output_dir / f"policy_{args.model_type}_best.pt"

    for epoch in range(args.epochs):
        # -- Training Step --
        model.train()
        total_train_loss = 0.0
        for xb, yb in train_loader:
            xb, yb = xb.to(device), yb.to(device)
            pred = model(xb)
            loss = loss_fn(pred, yb)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total_train_loss += loss.item()
        avg_train_loss = total_train_loss / len(train_loader)
        history["train_loss"].append(avg_train_loss)

        # -- Validation Step --
        model.eval()
        total_val_loss = 0.0
        with torch.no_grad():
            for xb, yb in val_loader:
                xb, yb = xb.to(device), yb.to(device)
                pred = model(xb)
                loss = loss_fn(pred, yb)
                total_val_loss += loss.item()
        avg_val_loss = total_val_loss / len(val_loader)
        history["val_loss"].append(avg_val_loss)

        logging.info(f"Epoch {epoch+1:03d}/{args.epochs} | Train Loss: {avg_train_loss:.6f} | Val Loss: {avg_val_loss:.6f}")

        # -- Checkpointing: Save the best model --
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            torch.save({
                "state_dict": model.state_dict(),
                "model_type": args.model_type,
                "input_dim": X_train.shape[1],
                "output_dim": y_train.shape[1],
                "X_mean": X_mean, "X_std": X_std,
                "y_mean": y_mean, "y_std": y_std,
                "best_val_loss": best_val_loss,
                "epoch": epoch + 1,
            }, model_save_path)
            logging.info(f"  -> New best model saved to {model_save_path} (Val Loss: {best_val_loss:.6f})")
            patience_counter = 0 # Reset counter
        else:
            patience_counter += 1 # Increment counter

        if patience_counter >= patience:
            logging.info(f"Validation loss did not improve for {patience} epochs. Stopping early.")
            break # Exit the training loop

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