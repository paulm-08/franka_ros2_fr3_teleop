import pickle
import numpy as np

# --- Configuration ---
# Update this path to your specific .pkl file
DATASET_PATH = "/home/user/franka_ros2_ws/data/processed_datasets/dataset_single_test.pkl"

# Index for the arm joint positions within the state vector (Q_t)
# (Your configuration: indices 24 up to, but not including, 31)
ARM_STATE_SLICE = slice(24, 31) 

# Index for the arm action deltas within the action vector (A_t)
ARM_ACTION_SLICE = slice(0, 7)

# Select a trajectory and starting timestep to check
TRAJECTORY_INDEX = 0
START_TIMESTEP = 50 # Start check from step 50
NUM_STEPS_TO_CHECK = 10 # Check steps 50, 51, and 52

# --- Data Loading and Verification ---
print(f"Loading data from: {DATASET_PATH}")
try:
    with open(DATASET_PATH, "rb") as f:
        # data is expected to be a list of dictionaries (trajectories)
        data = pickle.load(f)
except FileNotFoundError:
    print(f"ERROR: Dataset file not found at {DATASET_PATH}")
    exit()
except Exception as e:
    print(f"ERROR loading pickle file: {e}")
    exit()

if not data or not isinstance(data, list) or not isinstance(data[TRAJECTORY_INDEX], dict):
    print("ERROR: Data structure not as expected (should be list of dicts).")
    exit()

# Select the trajectory
trajectory = data[TRAJECTORY_INDEX]
print(f"\n--- Analysis for Trajectory Index {TRAJECTORY_INDEX} ---")

# Convert arrays to numpy for easier slicing and calculation
state_t = np.array(trajectory["state_t"])
action_t = np.array(trajectory["action_t"])

print(f"Full State Shape (T, D_state): {state_t.shape}")
print(f"Full Action Shape (T, D_action): {action_t.shape}")
print("-" * 120)

# Set print options for clean array output
np.set_printoptions(formatter={'float': '{: 10.8f}'.format}, linewidth=120)

# Iterate through the selected timesteps (t)
for t in range(START_TIMESTEP, START_TIMESTEP + NUM_STEPS_TO_CHECK):
    if t + 1 >= len(state_t):
        print(f"Skipping timestep {t}: Not enough future steps remaining.")
        break
    
    # 1. Extract joint positions and action
    Q_t = state_t[t][ARM_STATE_SLICE]
    Q_t_plus_1 = state_t[t + 1][ARM_STATE_SLICE]
    A_t_recorded = action_t[t][ARM_ACTION_SLICE]
    
    # 2. Calculate the Ground Truth Delta (Q_t+1 - Q_t)
    delta_Q_calculated = Q_t_plus_1 - Q_t
    
    # 3. Calculate the difference (error)
    error = delta_Q_calculated - A_t_recorded
    max_absolute_error = np.max(np.abs(error))
    
    print(f"\nVerification at Timestep t={t} (Joints J1 to J7):")
    print(f"  Q_t (Current Joint Angles):     {np.array2string(Q_t, separator=', ')}")
    print(f"  Q_t+1 (Next Joint Angles):      {np.array2string(Q_t_plus_1, separator=', ')}")
    print(f"  Max Absolute Error (should be ~0): {max_absolute_error:.10f}")
    
    # Print comparison table
    print(f"  {'Joint Index':<12} | {'Q_t+1 - Q_t (Calculated Delta)':<32} | {'A_t (Recorded Action)':<25} | {'Error (A_t - Delta)':<20}")
    print(f"  {'-'*10:<12} | {'-'*32:<32} | {'-'*25:<25} | {'-'*20:<20}")
    
    for i in range(7):
        print(f"  {i:<12} | {delta_Q_calculated[i]:<32.8f} | {A_t_recorded[i]:<25.8f} | {error[i]:<20.8f}")

    if max_absolute_error > 1e-5:
        print("\nWARNING: Large error found. This indicates an inconsistency between state difference and recorded action.")
        
print("-" * 120)
print("\nVerification complete.")