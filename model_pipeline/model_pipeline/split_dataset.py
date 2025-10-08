import numpy as np
from sklearn.model_selection import train_test_split
import os
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def main():
    # Load processed multimodal dataset
    data_path = "data/processed/dataset_features.npz"
    data = np.load(data_path)
    
    # Retrieve available arrays
    tactile_t = data["tactile_t"]
    joints_t = data["joints_t"]
    delta_q = data["delta_q"]
    
    # Visual features may or may not exist (for backward compatibility)
    visual_t = data.get("visual_t", None)

    logging.info(f"Tactile: {tactile_t.shape}, Joints: {joints_t.shape}, Visual: {None if visual_t is None else visual_t.shape}")

    # === Concatenate modalities ===
    # Input = tactile features + visual features + current joint angles
    if visual_t is not None:
        X = np.concatenate([tactile_t, visual_t, joints_t], axis=-1)
    else:
        X = np.concatenate([tactile_t, joints_t], axis=-1)
    
    y = delta_q  # Target = next-frame joint angles

    logging.info(f"Loaded dataset: X={X.shape}, y={y.shape}")

    # === Split train/test ===
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    logging.info(f"Train split: X={X_train.shape}, y={y_train.shape}")
    logging.info(f"Test split: X={X_test.shape}, y={y_test.shape}")

    # === Save result ===
    os.makedirs("data/processed", exist_ok=True)
    np.savez(
        "data/processed/dataset_final.npz",
        X_train=X_train, y_train=y_train,
        X_test=X_test, y_test=y_test
    )

    logging.info("✅ Split dataset into train/test → dataset_final.npz")

if __name__ == "__main__":
    main()
