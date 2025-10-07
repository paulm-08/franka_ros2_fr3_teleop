# scripts/preprocess_dataset.py
import numpy as np
from sklearn.model_selection import train_test_split

def main():
    data = np.load("data/processed/dataset_features.npz")
    tactile, actions = data["tactile"], data["actions"]

    # Normalize
    tactile = (tactile - tactile.mean(0)) / (tactile.std(0) + 1e-8)

    # Train/test split
    X_train, X_test, y_train, y_test = train_test_split(tactile, actions, test_size=0.2)
    np.savez("data/processed/dataset_final.npz",
             X_train=X_train, X_test=X_test,
             y_train=y_train, y_test=y_test)
    print("✅ Preprocessed dataset → dataset_final.npz")

if __name__ == "__main__":
    main()
