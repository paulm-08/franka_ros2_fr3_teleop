#!/usr/bin/env python3
import numpy as np
import torch
import matplotlib.pyplot as plt
import logging, os, argparse
from model_pipeline.train import PolicyNet
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def to_np(tensor):
    return tensor.detach().cpu().numpy()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rollout", action="store_true", help="Evaluate in closed-loop mode (recursive predictions)")
    parser.add_argument("--horizon", type=int, default=100, help="Number of rollout steps (ignored if longer than test set)")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    logging.info(f"Device: {device}")

    # === Load data ===
    data = np.load("data/processed/dataset_final.npz")
    if "X_test" not in data or "y_test" not in data:
        raise RuntimeError("dataset_final.npz must contain X_test and y_test")
    X_test = torch.tensor(data["X_test"], dtype=torch.float32, device=device)
    y_test = torch.tensor(data["y_test"], dtype=torch.float32, device=device)

    # === Load model + normalization ===
    checkpoint = torch.load("data/models/policy.pt", weights_only=False, map_location=device)
    model = PolicyNet(checkpoint["input_dim"], checkpoint["output_dim"]).to(device)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()

    X_mean = torch.tensor(checkpoint["X_mean"], dtype=torch.float32, device=device)
    X_std  = torch.tensor(checkpoint["X_std"], dtype=torch.float32, device=device)
    y_mean = torch.tensor(checkpoint["y_mean"], dtype=torch.float32, device=device)
    y_std  = torch.tensor(checkpoint["y_std"], dtype=torch.float32, device=device)

    # joint limits
    if "joint_min" in checkpoint and "joint_max" in checkpoint:
        joint_min = torch.tensor(checkpoint["joint_min"], dtype=torch.float32, device=device)
        joint_max = torch.tensor(checkpoint["joint_max"], dtype=torch.float32, device=device)
    else:
        joint_min = torch.full((y_test.shape[1],), -np.pi, dtype=torch.float32, device=device)
        joint_max = torch.full((y_test.shape[1],), np.pi, dtype=torch.float32, device=device)

    JOINT_START = X_test.shape[1] - y_test.shape[1]

    os.makedirs("data/debug", exist_ok=True)

    # === ONE-STEP EVAL ===
    if not args.rollout:
        with torch.no_grad():
            X_test_norm = (X_test - X_mean) / X_std
            delta_q_pred_norm = model(X_test_norm)
            delta_q_pred = delta_q_pred_norm * y_std + y_mean

        # current joint angles are the last features in X_test
        q_current = X_test[:, JOINT_START:]
        q_pred = q_current + delta_q_pred
        q_pred = torch.clamp(q_pred, min=joint_min, max=joint_max)

        # ground truth: if y_test already contains delta_q, compute q_true accordingly
        if checkpoint.get("target_type", "absolute") == "delta":
            q_true = q_current + y_test
        else:
            q_true = y_test

        # metrics
        q_pred_np = to_np(q_pred)
        q_true_np = to_np(q_true)
        mse = mean_squared_error(q_true_np, q_pred_np)
        mae = mean_absolute_error(q_true_np, q_pred_np)
        r2 = r2_score(q_true_np, q_pred_np)

        logging.info(f"✅ One-step eval complete | MSE={mse:.6f} | MAE={mae:.6f} | R²={r2:.4f}")

        # === Plots ===
        plt.figure(figsize=(6,6))
        plt.scatter(q_true_np[:,0], q_pred_np[:,0], alpha=0.5)
        mn, mx = q_true_np[:,0].min(), q_true_np[:,0].max()
        plt.plot([mn, mx], [mn, mx], "r--")
        plt.xlabel("Ground Truth"); plt.ylabel("Predicted")
        plt.title("Predicted vs GT (first joint)")
        plt.savefig("data/debug/eval_scatter.png"); plt.close()

        errors = q_pred_np - q_true_np
        plt.figure(figsize=(8,4))
        plt.hist(errors.flatten(), bins=50, alpha=0.7)
        plt.xlabel("Prediction Error"); plt.ylabel("Frequency")
        plt.title("Error Distribution")
        plt.savefig("data/debug/eval_error_hist.png"); plt.close()

        per_dim_mse = ((q_pred_np - q_true_np) ** 2).mean(axis=0)
        plt.figure(figsize=(10,4))
        plt.bar(np.arange(len(per_dim_mse)), per_dim_mse)
        plt.xlabel("Joint Index"); plt.ylabel("MSE")
        plt.title("Per-Joint MSE")
        plt.savefig("data/debug/eval_per_joint_mse.png"); plt.close()

    # === CLOSED-LOOP ROLLOUT ===
    else:
        rollout_steps = min(args.horizon, len(X_test) - 1)
        joint_dim = y_test.shape[1]
        logging.info(f"🔁 Closed-loop rollout for {rollout_steps} steps")

        preds, gts = [], []
        x_cur = X_test[0].clone().to(device)

        for t in range(rollout_steps):
            q_cur = x_cur[JOINT_START:].clone()

            x_norm = (x_cur - X_mean) / X_std
            with torch.no_grad():
                delta_q_pred_norm = model(x_norm.unsqueeze(0))
                delta_q_pred = (delta_q_pred_norm * y_std + y_mean).squeeze(0)

            q_next_pred = q_cur + delta_q_pred
            q_next_pred = torch.clamp(q_next_pred, min=joint_min, max=joint_max)

            preds.append(to_np(q_next_pred))
            gts.append(to_np(y_test[t] if checkpoint.get("target_type","absolute")=="absolute" 
                             else q_cur + y_test[t]))

            # update input for next iteration
            x_next = x_cur.clone()
            x_next[JOINT_START:] = q_next_pred
            x_cur = x_next

        pred_np = np.stack(preds)
        gt_np = np.stack(gts)
        pred_np = pred_np.reshape(rollout_steps, -1)
        gt_np = gt_np.reshape(rollout_steps, -1)

        mse = mean_squared_error(gt_np, pred_np)
        mae = mean_absolute_error(gt_np, pred_np)
        r2 = r2_score(gt_np, pred_np)
        logging.info(f"✅ Rollout complete | MSE={mse:.6f} | MAE={mae:.6f} | R²={r2:.4f}")

        drift_per_step = np.sqrt(((pred_np - gt_np)**2).mean(axis=1))
        drift_per_joint = np.sqrt(((pred_np - gt_np)**2).mean(axis=0))

        # === Plots ===
        plt.figure(figsize=(8,4))
        plt.plot(drift_per_step)
        plt.xlabel("Step"); plt.ylabel("RMSE per step")
        plt.title("Prediction Drift Over Time")
        plt.savefig("data/debug/eval_rollout_drift_over_time.png"); plt.close()

        plt.figure(figsize=(10,4))
        plt.bar(np.arange(len(drift_per_joint)), drift_per_joint)
        plt.xlabel("Joint Index"); plt.ylabel("RMSE")
        plt.title("Per-Joint Rollout Drift")
        plt.savefig("data/debug/eval_rollout_drift_per_joint.png"); plt.close()

        plt.figure(figsize=(12,6))
        for j in range(joint_dim):
            plt.plot(gt_np[:, j], 'k--', alpha=0.5)
            plt.plot(pred_np[:, j], label=f"Joint {j}")
        plt.xlabel("Step"); plt.ylabel("Joint angle (rad)")
        plt.title("Joint Trajectories: GT vs Predicted")
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.tight_layout()
        plt.savefig("data/debug/eval_rollout_joint_trajectories.png"); plt.close()

        np.savez("data/debug/rollout_results.npz",
                 pred=pred_np, gt=gt_np,
                 drift_per_step=drift_per_step, drift_per_joint=drift_per_joint,
                 mse=mse, mae=mae, r2=r2)
        logging.info("💾 Saved rollout predictions and metrics → data/debug/rollout_results.npz")

if __name__ == "__main__":
    main()
