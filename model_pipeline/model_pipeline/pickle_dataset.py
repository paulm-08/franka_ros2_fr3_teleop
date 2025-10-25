import pickle

with open("/home/user/franka_ros2_ws/data/processed_datasets/processed_dataset7.pkl", "rb") as f:
    data = pickle.load(f)

print("Keys:", data[0].keys())
print("state_t shape:", data[0]["state_t"].shape)
print("X_mean shape:", data[0].get("X_mean", None).shape if "X_mean" in data[0] else "missing")
