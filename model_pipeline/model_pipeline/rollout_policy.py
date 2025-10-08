#!/usr/bin/env python3
import numpy as np
import torch
import cv2
import logging, os, argparse
from model_pipeline.train import PolicyNet
from model_pipeline.visual_embedder import VisualEmbedder

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def to_np(tensor):
    return tensor.detach().cpu().numpy()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--use_live_visual", action="store_true",
                        help="If set, attempt to compute visual embeddings online (requires camera/images).")
    parser.add_argument("--horizon", type=int, default=200)
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    logging.info(f"Device: {device}")

    # Load data
    data = np.load("data/processed/dataset_final.npz")
    X_test = torch.tensor(data["X_test"], dtype=torch.float32, device=device)
    y_test = torch.tensor(data["y_test"], dtype=torch.float32, device=device)

    # Load model/checkpoint
    checkpoint = torch.load("data/models/policy.pt", weights_only=False, map_location=device)
    model = PolicyNet(checkpoint["input_dim"], checkpoint["output_dim"]).to(device)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()

    X_mean = torch.tensor(checkpoint["X_mean"], dtype=torch.float32, device=device)
    X_std = torch.tensor(checkpoint["X_std"], dtype=torch.float32, device=device)
    y_mean = torch.tensor(checkpoint["y_mean"], dtype=torch.float32, device=device)
    y_std = torch.tensor(checkpoint["y_std"], dtype=torch.float32, device=device)
    if "joint_min" in checkpoint and "joint_max" in checkpoint:
        joint_min = torch.tensor(checkpoint["joint_min"], dtype=torch.float32, device=device)
        joint_max = torch.tensor(checkpoint["joint_max"], dtype=torch.float32, device=device)
    else:
        joint_min = torch.full((y_test.shape[1],), -np.pi, dtype=torch.float32, device=device)
        joint_max = torch.full((y_test.shape[1],), np.pi, dtype=torch.float32, device=device)

    # If you want to compute visual embeddings online, instantiate embedder (optional)
    if args.use_live_visual:
        embedder = VisualEmbedder(backbone="resnet18", device=device, pretrained=True, out_dim=256)
        logging.info("Visual embedder (online) initialized.")

    joint_dim = y_test.shape[1]
    JOINT_START = X_test.shape[1] - joint_dim

    # Start rollout from first test frame
    x_cur = X_test[0].clone().to(device)
    q_cur = x_cur[JOINT_START:].clone()  # current joint angles

    max_steps = min(args.horizon, len(X_test)-1)
    logging.info(f"Running interactive rollout for {max_steps} steps. Press 'q' to quit early.")

    for t in range(max_steps):
        x_norm = (x_cur - X_mean) / X_std

        with torch.no_grad():
            y_pred_norm = model(x_norm.unsqueeze(0))
            delta_q_pred = (y_pred_norm * y_std + y_mean).view(-1)  # Δq
            q_next = q_cur + delta_q_pred  # qₜ₊₁ = qₜ + Δq
            q_next_clamped = torch.clamp(q_next, min=joint_min, max=joint_max)

        print(f"[Step {t}] pred_clamped Δq: {to_np(delta_q_pred).round(3)}  |  gt Δq: {to_np(y_test[t]).round(3)}")

        # Optional visualization
        canvas = np.ones((200, 700, 3), dtype=np.uint8) * 255
        cv2.putText(canvas, f"Step {t}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,0), 2)
        for j in range(min(10, joint_dim)):
            dq_pred = delta_q_pred[j].item()
            dq_gt = y_test[t, j].item()
            text = f"ΔJ{j}: P={dq_pred:.3f} GT={dq_gt:.3f}"
            cv2.putText(canvas, text, (10, 60 + j*18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,255), 1)
        cv2.imshow("Rollout (press q to stop)", canvas)
        k = cv2.waitKey(50) & 0xFF
        if k == ord("q"):
            break

        # Prepare next input
        q_cur = q_next_clamped.clone()
        x_next = x_cur.clone()
        x_next[JOINT_START:] = q_cur
        x_cur = x_next

    cv2.destroyAllWindows()
    logging.info("Done.")

if __name__ == "__main__":
    main()
