import pickle
import numpy as np

# --- Find the outlier ---
with open('/home/user/franka_ros2_ws/data/processed/tube_insertion_all.pkl', 'rb') as f:
    all_trajectories = pickle.load(f)

max_val = -1
outlier_traj_idx = -1
outlier_frame_idx = -1

for i, traj in enumerate(all_trajectories):
    max_in_traj = np.max(traj['delta_q'])
    if max_in_traj > max_val:
        max_val = max_in_traj
        outlier_traj_idx = i
        # Find the frame and joint within the trajectory
        outlier_frame_idx = np.unravel_index(np.argmax(traj['delta_q']), traj['delta_q'].shape)

print(f"Found max delta_q of {max_val:.4f} in trajectory index: {outlier_traj_idx}")
print(f"It occurs at frame {outlier_frame_idx[0]}, joint {outlier_frame_idx[1]}")