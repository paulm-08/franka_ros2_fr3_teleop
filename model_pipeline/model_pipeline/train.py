import numpy as np, torch, torch.nn as nn, torch.optim as optim
import os, logging, matplotlib.pyplot as plt

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

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
    X_train = torch.tensor(data["X_train"], dtype=torch.float32)
    y_train = torch.tensor(data["y_train"], dtype=torch.float32)

    model = PolicyNet(X_train.shape[1], y_train.shape[1])
    optimiz = optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()

    losses = []
    for epoch in range(100):  # more epochs
        pred = model(X_train)
        loss = loss_fn(pred, y_train)
        optimiz.zero_grad(); loss.backward(); optimiz.step()
        losses.append(loss.item())
        logging.info(f"Epoch {epoch+1}: loss={loss.item():.4f}")

    os.makedirs("data/models", exist_ok=True)
    torch.save(model.state_dict(), "data/models/policy.pt")
    logging.info("✅ Trained dummy policy → data/models/policy.pt")

    # Save loss curve
    os.makedirs("data/debug", exist_ok=True)
    plt.plot(losses)
    plt.xlabel("Epoch"); plt.ylabel("MSE Loss"); plt.title("Training Loss")
    plt.savefig("data/debug/train_loss.png")
    logging.info("📉 Saved training loss curve → data/debug/train_loss.png")

if __name__ == "__main__":
    main()
