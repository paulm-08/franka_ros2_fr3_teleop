# scripts/build_dataset.py
import os, json, numpy as np

def main():
    # Dummy example: just create a "dataset" with random tactile + actions
    dataset = {
        "tactile": np.random.rand(100, 3, 32, 32),  # 100 steps, 3 sensors, 32x32
        "actions": np.random.rand(100, 7),          # 7-DoF robot
    }
    os.makedirs("data/processed", exist_ok=True)
    np.savez("data/processed/dataset_raw.npz", **dataset)
    print("✅ Built dummy dataset: data/processed/dataset_raw.npz")

if __name__ == "__main__":
    main()
