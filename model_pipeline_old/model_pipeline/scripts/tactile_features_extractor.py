# scripts/tactile_feature_extractor.py
import numpy as np
from sklearn.decomposition import PCA

def main():
    data = np.load("data/processed/dataset_raw.npz")
    tactile = data["tactile"].reshape(100, -1)  # flatten
    pca = PCA(n_components=10).fit(tactile)
    tactile_features = pca.transform(tactile)
    np.savez("data/processed/dataset_features.npz",
             tactile=tactile_features, actions=data["actions"])
    print("✅ Extracted tactile features → dataset_features.npz")

if __name__ == "__main__":
    main()
