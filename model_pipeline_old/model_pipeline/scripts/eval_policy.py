# scripts/eval_policy.py
import numpy as np, torch
from train_bc import PolicyNet

def main():
    data = np.load("data/processed/dataset_final.npz")
    X_test, y_test = torch.tensor(data["X_test"], dtype=torch.float32), torch.tensor(data["y_test"], dtype=torch.float32)

    model = PolicyNet(X_test.shape[1], y_test.shape[1])
    model.load_state_dict(torch.load("data/processed/policy.pt"))
    model.eval()

    with torch.no_grad():
        pred = model(X_test)
        mse = ((pred - y_test)**2).mean().item()
    print(f"✅ Test MSE = {mse:.4f}")

if __name__ == "__main__":
    main()
