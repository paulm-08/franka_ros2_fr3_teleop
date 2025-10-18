from pathlib import Path
import os

def find_workspace_root(start_path: Path) -> Path:
    """
    Searches upwards from the start_path to find the workspace root.
    The workspace root is identified as the directory that contains the 'src' folder.
    """
    current_path = start_path.resolve()
    while True:
        # Check if the 'src' directory exists in the current path
        if (current_path / 'src').is_dir():
            return current_path
        
        # Move up to the parent directory
        parent_path = current_path.parent
        
        # If we have reached the filesystem root and haven't found 'src', raise an error
        if parent_path == current_path:
            raise FileNotFoundError(
                "Could not find workspace root. Make sure your project is in a "
                "standard ROS 2 workspace structure with a 'src' directory."
            )
        current_path = parent_path

# --- Dynamically Find Project Roots ---
# This script is now smart enough to work whether it's run from 'src/' or 'install/'

# 1. WORKSPACE_ROOT: The root of your entire workspace (e.g., '.../franka_ros2_ws/')
# We find this by searching upwards from this file's location for the 'src' landmark.
try:
    WORKSPACE_ROOT = find_workspace_root(Path(__file__).parent)
except FileNotFoundError as e:
    # Handle the error gracefully if the script fails to find the root
    print(f"[ERROR] Path resolution failed: {e}")
    # Set a fallback or exit, depending on desired behavior
    WORKSPACE_ROOT = Path.cwd() # Fallback to current working directory, with a warning
    print(f"[WARNING] Falling back to current working directory as WORKSPACE_ROOT: {WORKSPACE_ROOT}")


# 2. SRC_ROOT: The root of your git repository ('.../franka_ros2_ws/src/')
SRC_ROOT = WORKSPACE_ROOT / "src"

# --- Define All Other Project Paths Relative to the Workspace Root ---

# Configuration Path
CONFIG_DIR = WORKSPACE_ROOT / "config"
DEFAULT_CONFIG_PATH = CONFIG_DIR / "config.yaml"

# Data Paths
DATA_DIR = WORKSPACE_ROOT / "data"
RAW_DATA_DIR = DATA_DIR / "recorded_data"
PROCESSED_DATA_DIR = DATA_DIR / "processed_datasets"
YOLO_DATA_DIR = DATA_DIR / "yolo_datasets"
YOLO_ANNOTATIONS_DIR = YOLO_DATA_DIR / "yolo_annotations"
YOLO_IMAGES_DIR = YOLO_DATA_DIR / "sampled_images"

# Model Paths
MODELS_DIR = WORKSPACE_ROOT / "models"
POLICY_MODELS_DIR = MODELS_DIR / "policy_models"
YOLO_MODELS_DIR = WORKSPACE_ROOT / "runs"

# --- Helper function to print all paths for debugging ---
def log_all_paths():
    """Prints all defined paths to verify they are correct."""
    print("="*50)
    print(" Dynamically Determined Paths ".center(50, '-'))
    print("="*50)
    print(f"WORKSPACE_ROOT:    {WORKSPACE_ROOT}")
    print(f"SRC_ROOT:          {SRC_ROOT}")
    print(f"CONFIG_DIR:        {CONFIG_DIR}")
    print(f"RAW_DATA_DIR:      {RAW_DATA_DIR}")
    print(f"MODELS_DIR:        {MODELS_DIR}")
    print("="*50)

if __name__ == '__main__':
    # You can run this file directly to check if the paths are correct
    # On Windows: python -m model_pipeline.paths
    # On Linux:   ros2 run model_pipeline paths
    log_all_paths()

