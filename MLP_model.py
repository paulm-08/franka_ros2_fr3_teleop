import torch
import torch.nn as nn

class PolicyMLP(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_sizes=[128, 64]):
        super().__init__()
        layers = []
        dims = [state_dim] + hidden_sizes + [action_dim]
        for i in range(len(dims)-2):
            layers.append(nn.Linear(dims[i], dims[i+1]))
            layers.append(nn.ReLU())
        layers.append(nn.Linear(dims[-2], dims[-1]))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)

model = PolicyMLP(state_dim=X.shape[1], action_dim=Y.shape[1])
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
loss_fn = nn.MSELoss()
epochs = 100
dataloader = torch.utils.data.DataLoader(dataset, batch_size=32, shuffle=True)
for epoch in range(epochs):
    for batch_x, batch_y in dataloader:
        pred = model(batch_x)
        loss = loss_fn(pred, batch_y)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
