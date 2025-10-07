# scripts/train_bc.py
import numpy as np, torch, torch.nn as nn, torch.optim as optim

class PolicyNet(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 64), nn.ReLU(),
            nn.Linear(64, output_dim)
        )
    def forward(self, x): return self.net(x)

def main():
    data = np.load("data/processed/dataset_final.npz")
    X_train, y_train = torch.tensor(data["X_train"], dtype=torch.float32), torch.tensor(data["y_train"], dtype=torch.float32)

    model = PolicyNet(X_train.shape[1], y_train.shape[1])
    optimiz = optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()

    for epoch in range(5):  # dummy training
        pred = model(X_train)
        loss = loss_fn(pred, y_train)
        optimiz.zero_grad(); loss.backward(); optimiz.step()
        print(f"Epoch {epoch}: loss={loss.item():.4f}")

    torch.save(model.state_dict(), "data/processed/policy.pt")
    print("✅ Trained dummy policy → policy.pt")

if __name__ == "__main__":
    main()
