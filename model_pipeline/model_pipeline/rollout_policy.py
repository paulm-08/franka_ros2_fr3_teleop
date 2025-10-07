import numpy as np
import torch
import torch.nn as nn
import cv2
import os
from model_pipeline.train import PolicyNet

def load_model(input_dim, output_dim, model_path="data/models/policy.pt"):
    model = PolicyNet(input_dim, output_dim)
    model.load_state_dict(torch.load(model_path))
    model.eval()
    return model

def main():
    # Load dataset (use test split for rollout)
    data = np.load("data/processed/dataset_final.npz")
    X_test = torch.tensor(data["X_test"], dtype=torch.float32)
    y_test = torch.tensor(data["y_test"], dtype=torch.float32)

    # Load trained model
    model = load_model(X_test.shape[1], y_test.shape[1])

    print(f"✅ Loaded model and dataset. Rollout length = {len(X_test)}")

    for i, obs in enumerate(X_test):
        with torch.no_grad():
            pred = model(obs.unsqueeze(0)).squeeze(0).numpy()
        gt = y_test[i].numpy()

        # Print rollout step
        print(f"[Step {i}] obs={obs.numpy()} → pred={pred.round(2)} | gt={gt.round(2)}")

        # (Optional) Visualization with OpenCV
        canvas = np.ones((300, 500, 3), dtype=np.uint8) * 255
        cv2.putText(canvas, f"Step {i}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        for j, (p, g) in enumerate(zip(pred, gt)):
            cv2.putText(canvas, f"Joint {j}: P={p:.2f} G={g:.2f}",
                        (20, 80 + j*25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)

        cv2.imshow("Rollout", canvas)
        key = cv2.waitKey(200)  # advance automatically, 200ms per frame
        if key == ord("q"):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
