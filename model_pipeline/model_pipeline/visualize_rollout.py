import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import inquirer
from pathlib import Path
import sys

# --- Configuration ---
# NOTE: Make sure this path points to your directory containing the rollout files.
# It assumes a 'models/rollout' directory relative to where you run the script.
# You can also use an absolute path, e.g., Path("/home/user/my_project/models/rollout")
try:
    # Assuming you have a paths.py file like in your project
    from model_pipeline import paths
    ROLLOUT_DIR = paths.MODELS_DIR / "rollout"
except (ImportError, NameError):
    print("⚠️  Warning: 'paths' module not found. Using a default relative path './models/rollout'.")
    print("   Please edit the ROLLOUT_DIR variable if this is incorrect.")
    ROLLOUT_DIR = Path("./models/rollout")


def find_rollout_files(search_path: Path):
    """Searches a directory for rollout .npz files and returns their paths."""
    print(f"🔍 Searching for rollouts in: {search_path.resolve()}")
    if not search_path.is_dir():
        print(f"❌ Error: Directory not found at '{search_path.resolve()}'")
        return []
    # Use glob to find all files matching the pattern
    return sorted(list(search_path.glob("rollout_*.npz")))


def main():
    """Main function to select and plot a rollout trajectory."""
    rollout_files = find_rollout_files(ROLLOUT_DIR)

    if not rollout_files:
        print("\nNo rollout files found. Please run a real-world rollout first.")
        sys.exit(1)

    # Use inquirer to create an interactive list choice prompt
    questions = [
        inquirer.List(
            'filepath',
            # Show just the clean filename to the user
            message="Select a rollout file to plot",
            choices=[f.name for f in rollout_files],
        )
    ]

    answers = inquirer.prompt(questions)

    # Handle case where user exits with Ctrl+C
    if not answers:
        print("\nAborted.")
        sys.exit(0)

    # Reconstruct the full path from the chosen filename
    selected_filename = answers['filepath']
    selected_filepath = ROLLOUT_DIR / selected_filename

    # --- 1. Load Data ---
    print(f"\n--- Loading data from {selected_filename} ---")
    data = np.load(selected_filepath)
    proprio = data['proprioception']  # Shape: (num_steps, 23)
    timestamps = data['timestamps']
    print(f"Loaded {proprio.shape[0]} timesteps.")

    # --- 2. Parse Filename for Plotting Info ---
    # Example: "rollout_t_task_space.npz" -> ['rollout', 't', 'task', 'space']
    try:
        parts = selected_filename.removesuffix('.npz').split('_')
        model_type = parts[1].upper()
        control_mode = parts[2]
        title = f"Real Robot Trajectory ({model_type} | {control_mode.replace('_', ' ').title()})"
    except IndexError:
        # Fallback if filename format is unexpected
        model_type = "Unknown"
        control_mode = "task_space" if proprio.shape[1] == 23 else "joint_space"
        title = f"Real Robot Trajectory ({selected_filename})"


    # --- 3. Prepare Data for Plotting ---
    if control_mode == 'task_space':
        print("Processing task space data (Position + Euler Angles)...")
        plot_data = np.zeros((proprio.shape[0], 6))
        plot_data[:, :3] = proprio[:, :3]  # Position (x, y, z)

        # Convert quaternions to Euler angles and unwrap for continuity
        # Scipy expects quaternion as [x, y, z, w]
        euler_angles = R.from_quat(proprio[:, 3:7]).as_euler('xyz', degrees=False)
        plot_data[:, 3:] = np.unwrap(euler_angles, axis=0)

        labels = ["x", "y", "z", "roll", "pitch", "yaw"]
        ylabel = "Position (m) / Angle (rad)"

    else:  # joint_space
        print("Processing joint space data (Joint Angles)...")
        num_arm_joints = 7
        plot_data = proprio[:, :num_arm_joints]  # Plot only the 7 arm joints
        labels = [f"Joint {i+1}" for i in range(num_arm_joints)]
        ylabel = "Joint Angle (rad)"

    # --- 4. Plot Trajectory ---
    fig, ax = plt.subplots(figsize=(14, 7))
    time_axis = timestamps - timestamps[0]  # Time in seconds from start

    for i in range(plot_data.shape[1]):
        ax.plot(time_axis, plot_data[:, i], label=labels[i])

    ax.set(title=title, xlabel="Time (s)", ylabel=ylabel)
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0))
    ax.grid(True, linestyle='--')
    plt.tight_layout(rect=[0, 0, 0.9, 1]) # Adjust layout to make space for legend
    print("🚀 Displaying plot...")
    plt.show()


if __name__ == "__main__":
    main()