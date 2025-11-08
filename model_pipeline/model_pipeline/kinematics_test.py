import pickle
import numpy as np
import logging
from pathlib import Path
import inquirer
import subprocess
import tempfile
import os
import shutil
import time

# --- ROS 2 and Launch-specific imports ---
from ament_index_python.packages import get_package_share_directory

from model_pipeline import paths
# We can now import the solver and the new helper function
from model_pipeline.kinematics import KinematicsSolver, get_urdf_string_from_xacro
from model_pipeline.utils import find_pkl_files
import dex_retargeting.leap_hand_utils.leap_hand_utils as lhu


logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def get_urdf_string_from_xacro():
    """
    Replicates the logic from your launch file to generate the
    robot's URDF content as a string by executing the xacro command.
    """
    logging.info("Generating URDF from xacro (replicating launch file logic)...")
    
    xacro_path = shutil.which('xacro')
    if not xacro_path:
        logging.error("Could not find 'xacro' executable. Please ensure it is installed and in your PATH.")
        return None

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    
    command = [
        xacro_path, franka_xacro_file,
        'hand:=true', 'ee_id:=leap_hand', 'robot_ip:=none',
        'use_fake_hardware:=true', 'ros2_control:=true'
    ]
    
    try:
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        logging.info("Successfully generated URDF content.")
        return result.stdout
    except Exception as e:
        logging.error(f"Failed to run xacro command: {e}")
        return None

def main():
    # --- 1. Find and select the dataset to test ---
    pkl_choices = find_pkl_files(paths.PROCESSED_DATA_DIR)
    if not pkl_choices:
        logging.error(f"No .pkl datasets found in {paths.PROCESSED_DATA_DIR}.")
        return

    questions = [
        inquirer.List('dataset_pkl', 
                      message="Select the dataset (.pkl) to test kinematics on", 
                      choices=pkl_choices)
    ]
    try:
        answers = inquirer.prompt(questions)
        if not answers:
            logging.info("No selection made. Exiting.")
            return
        dataset_path = paths.WORKSPACE_ROOT / answers['dataset_pkl']
    except (KeyboardInterrupt, TypeError):
        logging.info("\nOperation cancelled.")
        return


    # --- 2. Initialize the Kinematics Solver ---
    
    # 2a. Get the URDF content by running the xacro command
    urdf_content = get_urdf_string_from_xacro()
    if not urdf_content:
        return

    urdf_temp_file = None
    try:
        # 2b. Save to a temporary file for Pinocchio to load
        # This creates a file with a unique name in the system's temp directory
        with tempfile.NamedTemporaryFile(delete=False, mode='w', suffix='.urdf', encoding='utf-8') as f:
            f.write(urdf_content)
            urdf_temp_file = Path(f.name)
        
        EE_FRAME_NAME = "fr3_link8"  # End-effector frame name
        TACTILE_FRAMES = [
            "index_tip_head", # Example name, REPLACE with your actual frame
            "middle_tip_head", # Example name, REPLACE with your actual frame
            "thumb_tip_head"     # Example name, REPLACE with your actual frame
        ]

        
        # 2c. Pass the PATH to the temporary file to the solver
        solver = KinematicsSolver(
            urdf_content=urdf_temp_file,
            end_effector_frame_name= EE_FRAME_NAME,
            tactile_frame_names= TACTILE_FRAMES
        )

    except Exception as e:
        logging.error(f"Failed to initialize solver: {e}")
        return
    finally:
        # 2d. Clean up the temporary file
        if urdf_temp_file and urdf_temp_file.exists():
            os.remove(urdf_temp_file)

    # --- 3. Load Data and Run Test ---
    logging.info(f"Loading data from {dataset_path}...")
    with open(dataset_path, "rb") as f:
        all_trajectories = pickle.load(f)
    
    traj = all_trajectories[0]
    joint_states = traj['state_t'][:,24:24+23]
    
    logging.info(f"--- Running Forward Kinematics on first 10 steps of trajectory 0 ---")
    np.set_printoptions(precision=4, suppress=True)
    time.sleep(2)  # Small delay before starting

    for i in range(min(500, len(joint_states))):
        arm_joints = joint_states[i, :7] 
        hand_joints = joint_states[i, 7:23] # Get the 16 hand joints

        q_full, poses = solver.get_all_poses(arm_joints, hand_joints)
        
        logging.info(f"Step {i}")
        logging.info(f"  > Arm Joints (rad):  {arm_joints}")
        logging.info(f"  > Hand Joints (rad): {hand_joints}") # <-- NEW LOG
        
        logging.info(f"  > EE Pose:           {poses['ee']}")
        for name, pose in poses.items():
            if name != 'ee':
                logging.info(f"  > {name}: {pose}")
        logging.info("") # Newline

        # --- Update the visualizer ---
        solver.update_visualizer(q_full)
        time.sleep(0.05)  # Small delay to visualize
        
    #     # --- Display the camera image ---
    #     img_path = Path(frame_path) / "color_image1.jpg"
    #     if img_path.exists():
    #         img = cv2.imread(str(img_path))
    #         cv2.imshow("Camera 1 View", img)
        
    #     key = cv2.waitKey(int(1000 / 20)) # Play at ~20 Hz
    #     if key == ord('q'):
    #         break
            
    # cv2.destroyAllWindows()
    # logging.info("Test complete.")


if __name__ == "__main__":
    main()