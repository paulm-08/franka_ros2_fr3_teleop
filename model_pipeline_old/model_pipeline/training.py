"""Small MLP training script for behavior cloning.

Assumes `X, Y` saved in an npz from preprocessing step.
"""
import argparse
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader


class PolicyMLP(nn.Module):
    def __init__(self, state_dim, action_dim, hidden=[128, 64]):
        super().__init__()
        layers = []
        dims = [state_dim] + hidden
        for i in range(len(dims)-1):
            layers.append(nn.Linear(dims[i], dims[i+1]))
            layers.append(nn.ReLU())
        layers.append(nn.Linear(dims[-1], action_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


def train_npz(npz_path, epochs=50, batch_size=64, lr=1e-3, out_path=None):
    data = np.load(npz_path)
    X = data['X'].astype(np.float32)
    Y = data['Y'].astype(np.float32)

    state_dim = X.shape[1]
    action_dim = Y.shape[1]

    ds = TensorDataset(torch.from_numpy(X), torch.from_numpy(Y))
    dataloader = DataLoader(ds, batch_size=batch_size, shuffle=True)
    model = PolicyMLP(state_dim, action_dim)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()
    for epoch in range(epochs):
        epoch_loss = 0.0
        for batch_x, batch_y in dataloader:
            pred = model(batch_x)
            loss = loss_fn(pred, batch_y)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item() * batch_x.size(0)
        epoch_loss /= len(ds)
        print(f"Epoch {epoch+1}/{epochs}, Loss: {epoch_loss:.6f}")
    if out_path is not None:
        torch.save(model.state_dict(), out_path)
        print(f"Saved trained model to {out_path}") 
    return model
