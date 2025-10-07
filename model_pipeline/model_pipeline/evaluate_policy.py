import numpy as np, torch, torch.nn as nn
import matplotlib.pyplot as plt
import logging, os
from model_pipeline.train import PolicyNet

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def main():
    data = np.load("data/processed/dataset_final.npz")
    X_test = torch.tensor(data["X_test"], dtype=torch.float32)
    y_test = torch.tensor(data["y_test"], dtype=torch.float32)

    model = PolicyNet(X_test.shape[1], y_test.shape[1])
    model.load_state_dict(torch.load("data/models/policy.pt"))
    model.eval()

    with torch.no_grad():
        pred = model(X_test)
        mse = ((pred - y_test) ** 2).mean().item()

    logging.info(f"✅ Evaluation complete. Test MSE: {mse:.4f}")

    # Plot predicted vs ground truth for first output dim
    os.makedirs("data/debug", exist_ok=True)
    plt.figure(figsize=(6,6))
    plt.scatter(y_test[:,0], pred[:,0], alpha=0.5)
    plt.plot([y_test[:,0].min(), y_test[:,0].max()],
             [y_test[:,0].min(), y_test[:,0].max()], "r--")
    plt.xlabel("Ground Truth"); plt.ylabel("Prediction")
    plt.title("Predicted vs. Ground Truth (first action dim)")
    plt.savefig("data/debug/eval_scatter.png")
    logging.info("📊 Saved evaluation scatter plot → data/debug/eval_scatter.png")

if __name__ == "__main__":
    main()
