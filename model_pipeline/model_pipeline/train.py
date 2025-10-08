#!/usr/bin/env python3
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import os, logging, matplotlib.pyplot as plt
import random
import argparse

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

class PolicyNet(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 64), nn.ReLU(),
            nn.Linear(64, output_dim)
        )
    def forward(self, x): return self.net(x)

def set_seed(seed=0):
    np.random.seed(seed)
    random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)

def main(
    data_path="data/processed/dataset_final.npz",
    epochs=100,
    batch_size=64,
    lr=1e-3,
    lambda_penalty=0.1,   # smaller default; tune
    joint_margin=0.05,    # margin fraction (5%)
    seed=0
):
    # Parse args for epochs
    parser = argparse.ArgumentParser()
    parser.add_argument("--epoch", type=int, default=100, help="Number of epochs")
    args = parser.parse_args()
    epochs = args.epoch

    device = "cuda" if torch.cuda.is_available() else "cpu"
    logging.info(f"Device: {device}")
    logging.info(f"Training for {epochs} epochs")
    
    set_seed(seed)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    logging.info(f"Device: {device}")

    data = np.load(data_path)
    X_train_np = data["X_train"]
    y_train_np = data["y_train"]
    X_test_np = data["X_test"]
    y_test_np = data["y_test"]

    # Derive robust joint limits from percentiles to avoid single-outlier expansion
    q_low = np.percentile(y_train_np, 1.0, axis=0)
    q_high = np.percentile(y_train_np, 99.0, axis=0)
    span = q_high - q_low
    margin_abs = np.maximum(np.abs(q_low) * joint_margin, span * joint_margin)
    joint_min = q_low - margin_abs
    joint_max = q_high + margin_abs
    logging.info(f"Derived joint limits (first 5): min={joint_min[:5]}, max={joint_max[:5]}")

    # Convert to tensors
    X_train = torch.tensor(X_train_np, dtype=torch.float32)
    y_train = torch.tensor(y_train_np, dtype=torch.float32)
    X_test = torch.tensor(X_test_np, dtype=torch.float32)
    y_test = torch.tensor(y_test_np, dtype=torch.float32)

    # Move to device later after normalization (we keep CPU tensors for mean/std saving)
    # Normalize (per-dimension)
    X_mean = X_train.mean(0, keepdims=True)
    X_std = X_train.std(0, keepdims=True) + 1e-9
    y_mean = y_train.mean(0)
    y_std = y_train.std(0) + 1e-9

    X_train_norm = (X_train - X_mean) / X_std
    y_train_norm = (y_train - y_mean) / y_std
    X_test_norm = (X_test - X_mean) / X_std
    y_test_norm = (y_test - y_mean) / y_std

    # Move tensors to device
    X_train_norm = X_train_norm.to(device)
    y_train_norm = y_train_norm.to(device)
    X_test_norm = X_test_norm.to(device)
    y_test_norm = y_test_norm.to(device)

    model = PolicyNet(X_train.shape[1], y_train.shape[1]).to(device)
    optimiz = optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    loader = DataLoader(TensorDataset(X_train_norm, y_train_norm), batch_size=batch_size, shuffle=True)
    losses = []

    joint_min_t = torch.tensor(joint_min, dtype=torch.float32, device=device)
    joint_max_t = torch.tensor(joint_max, dtype=torch.float32, device=device)
    y_mean_t = y_mean.to(device)
    y_std_t = y_std.to(device)

    for epoch in range(epochs):
        epoch_loss = 0.0
        num_batches = 0
        for Xb, yb in loader:
            pred_norm = model(Xb)  # normalized-space prediction
            mse_loss = loss_fn(pred_norm, yb)

            # Denormalize prediction to get actual joint-angle deltas (or angles)
            pred_denorm = pred_norm * y_std_t + y_mean_t  # shape (B, out_dim)

            # Soft penalty: squared violation outside [joint_min, joint_max]
            low_violation = torch.relu(joint_min_t - pred_denorm)
            high_violation = torch.relu(pred_denorm - joint_max_t)
            penalty = (low_violation**2 + high_violation**2).mean()

            loss = mse_loss #+ lambda_penalty * penalty

            optimiz.zero_grad()
            loss.backward()
            optimiz.step()

            epoch_loss += loss.item()
            num_batches += 1

        avg_loss = epoch_loss / max(1, num_batches)
        losses.append(avg_loss)
        logging.info(f"Epoch {epoch+1}/{epochs}: loss={avg_loss:.6f}")

    # Evaluate on test set (normalized MSE)
    with torch.no_grad():
        pred_test_norm = model(X_test_norm)
        mse_test = loss_fn(pred_test_norm, y_test_norm).item()
        pred_test_denorm = pred_test_norm * y_std_t + y_mean_t
        oob_low = (pred_test_denorm < joint_min_t).any(dim=1).float().mean().item()
        oob_high = (pred_test_denorm > joint_max_t).any(dim=1).float().mean().item()
    logging.info(f"📊 Final test (norm-MSE) = {mse_test:.6f}; OOB fraction low={oob_low:.3f}, high={oob_high:.3f}")

    # Save model + normalization + joint limits + meta
    os.makedirs("data/models", exist_ok=True)
    save_dict = {
        "state_dict": model.state_dict(),
        "input_dim": X_train.shape[1],
        "output_dim": y_train.shape[1],
        "X_mean": X_mean.cpu().numpy(),
        "X_std": X_std.cpu().numpy(),
        "y_mean": y_mean.cpu().numpy(),
        "y_std": y_std.cpu().numpy(),
        # "joint_min": joint_min.astype(np.float32),
        # "joint_max": joint_max.astype(np.float32),
        "target_type": "delta",   # IMPORTANT: model predicts delta_q
    }
    torch.save(save_dict, "data/models/policy.pt")
    logging.info("✅ Trained policy + metadata saved → data/models/policy.pt")

    # Save training curve
    os.makedirs("data/debug", exist_ok=True)
    plt.plot(losses)
    plt.xlabel("Epoch"); plt.ylabel("Loss"); plt.title("Training Loss")
    plt.savefig("data/debug/train_loss.png")
    logging.info("📉 Saved training loss curve → data/debug/train_loss.png")

if __name__ == "__main__":
    main()
