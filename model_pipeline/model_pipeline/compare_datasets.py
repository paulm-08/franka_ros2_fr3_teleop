import hashlib, numpy as np
import pickle, numpy as np

full = pickle.load(open("/home/user/franka_ros2_ws/data/processed_datasets/processed_dataset_joint_test1.pkl", "rb"))
small = pickle.load(open("/home/user/franka_ros2_ws/data/processed_datasets/processed_dataset_joint_test2.pkl", "rb"))

def agg_stats(trajs):
    S = np.concatenate([t['state_t'] for t in trajs], axis=0)
    G = np.concatenate([t['goal_t'] for t in trajs], axis=0)
    A = np.concatenate([t['action_t'] for t in trajs], axis=0)
    return {"state_mean": S.mean(0), "state_std": S.std(0), 
            "goal_mean": G.mean(0), "goal_std": G.std(0),
            "action_mean": A.mean(0), "action_std": A.std(0),
            "num_traj": len(trajs)}

s_full = agg_stats(full)
s_small = agg_stats(small)

print("Full: trajs", s_full["num_traj"], "samples", sum(len(t['state_t']) for t in full))
print("Small: trajs", s_small["num_traj"], "samples", sum(len(t['state_t']) for t in small))

# Compare a few blocks (tactile / arm proprio / visual start indices per your pipeline)
tactile_dim = 24
arm_dim = 7
hand_dim = 16
vis_dim = 116

def compare_block(name, mean_full, mean_small, std_full, std_small, start, dim):
    mf = mean_full[start:start+dim]; ms = mean_small[start:start+dim]
    sf = std_full[start:start+dim]; ss = std_small[start:start+dim]
    print(f"\n{name}: mean Δ (max abs) = {np.max(np.abs(mf-ms)):.4f}, std Δ (max abs) = {np.max(np.abs(sf-ss)):.4f}")

mean_f = s_full["state_mean"]; mean_s = s_small["state_mean"]
std_f  = s_full["state_std"];  std_s  = s_small["state_std"]

idx = 0
compare_block("Tactile", mean_f, mean_s, std_f, std_s, idx, tactile_dim); idx+=tactile_dim
compare_block("Arm proprio", mean_f, mean_s, std_f, std_s, idx, arm_dim); idx+=arm_dim
compare_block("Hand proprio", mean_f, mean_s, std_f, std_s, idx, hand_dim); idx+=hand_dim
# skip tactile poses if not used
compare_block("Visual", mean_f, mean_s, std_f, std_s, idx, vis_dim)

def traj_hash(traj):
    h = hashlib.sha256()
    # include shapes for robustness
    h.update(str(traj['state_t'].shape).encode())
    h.update(traj['state_t'].tobytes())
    h.update(traj['goal_t'].tobytes())
    h.update(traj['action_t'].tobytes())
    return h.hexdigest()

full_hashes = {traj_hash(t):i for i,t in enumerate(full)}
small_hashes = {traj_hash(t):i for i,t in enumerate(small)}

common = set(full_hashes.keys()) & set(small_hashes.keys())
print("common trajectories:", len(common), " / ", len(small))
if len(common) < len(small):
    print("Some subset trajectories are NOT byte-identical to ones in the full processed dataset.")

import matplotlib.pyplot as plt

# pick index 0 in small
t_small = small[0]
# try to find an identical state sequence in full (naive)
for i,t in enumerate(full):
    if t['state_t'].shape == t_small['state_t'].shape and np.allclose(t['state_t'], t_small['state_t']):
        print("Found identical trajectory at full index", i)
        t_full = t
        break
else:
    print("No identical trajectory found; they differ")

# plot visual dims 0..5
plt.figure(figsize=(10,4))
plt.plot(t_small['state_t'][:, -vis_dim:], label='small last visual dims (first 5)') # adjust slicing as needed
plt.title("visual features (small) — first frames")
plt.show()
