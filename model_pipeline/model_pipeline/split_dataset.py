import numpy as np
from sklearn.model_selection import train_test_split
import os
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

def main():
    data = np.load("data/processed/dataset_features.npz")
    X, y = data["tactile"], data["actions"]

    logging.info(f"Loaded dataset: X={X.shape}, y={y.shape}")

    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42
    )

    logging.info(f"Train split: X={X_train.shape}, y={y_train.shape}")
    logging.info(f"Test split: X={X_test.shape}, y={y_test.shape}")

    os.makedirs("data/processed", exist_ok=True)
    np.savez("data/processed/dataset_final.npz",
             X_train=X_train, y_train=y_train,
             X_test=X_test, y_test=y_test)

    logging.info("✅ Split dataset into train/test → dataset_final.npz")

if __name__ == "__main__":
    main()
