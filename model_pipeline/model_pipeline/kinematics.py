import pinocchio as pin
import numpy as np
from pathlib import Path
import logging
import os
import shutil
from ament_index_python.packages import get_package_share_directory
import subprocess

# --- NEW IMPORTS for 3D Visualizer ---
import meshcat
from pinocchio.visualize import MeshcatVisualizer

from model_pipeline import paths
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

class KinematicsSolver:
    """
    A simple forward kinematics solver using Pinocchio, loaded from a
    dynamically generated URDF. Now includes a 3D visualizer.
    """
    def __init__(self, urdf_content, end_effector_frame_name, tactile_frame_names: list = None, visualize: bool = True):
        """
        Initializes the solver by loading the robot model from its URDF content.

        Args:
            urdf_content (str or Path): The full XML content of the robot's URDF or a path to it.
            end_effector_frame_name (str): The name of the end-effector link (e.g., 'fr3_hand_tcp').
            tactile_frame_names (list, optional): A list of names for the tactile sensor frames.
        """
        try:
            # --- THE FIX ---
            # Cast the input (which is a Path object) to a string,
            # as Pinocchio's bindings expect a string file path, not a Path object.
            urdf_path_str = str(urdf_content)
            
            # Load the model directly from the URDF string/path
            self.model = pin.buildModelFromUrdf(urdf_path_str)
            self.data = self.model.createData()
            
            # --- Load Visual Model for Meshcat ---
            
            # 1. Define the package directories where meshes (.stl, .dae) are stored
            #    This is the crucial step you were missing.
            package_dirs = [
                os.path.join(get_package_share_directory('franka_description')),
                # If your leap_hand meshes are in a different package, add its path here:
                # os.path.join(get_package_share_directory('leap_hand_description'))
            ]
            
            # 2. Load the visual model, telling Pinocchio where to find the packages
            if visualize:
                logging.info("Loading visual model for Meshcat visualization...")
                self.visual_model = pin.buildGeomFromUrdf(
                    self.model, 
                    urdf_path_str, 
                    pin.GeometryType.VISUAL, 
                    package_dirs=package_dirs
                )
            else:
                self.visual_model = None
            # self.visual_data = self.visual_model.createData()
            
            # --- Get the ID for the main end-effector ---
            self.ee_frame_id = self.model.getFrameId(end_effector_frame_name)
            if not self.model.existFrame(end_effector_frame_name):
                logging.warning(f"End-effector frame '{end_effector_frame_name}' not found!")
                self.ee_frame_id = self.model.nframes - 1
                logging.warning(f"Using last frame '{self.model.frames[self.ee_frame_id].name}' as end-effector.")

            # --- Get IDs for all tactile sensor frames ---
            self.tactile_frame_ids = {}
            if tactile_frame_names:
                for name in tactile_frame_names:
                    if self.model.existFrame(name):
                        self.tactile_frame_ids[name] = self.model.getFrameId(name)
                        logging.info(f"  > Found tactile frame: '{name}'")
                    else:
                        logging.warning(f"  > Tactile frame '{name}' not found in URDF!")

            logging.info(f"✅ KinematicsSolver initialized for frame '{self.model.frames[self.ee_frame_id].name}'.")
            logging.info(f"   Model has {self.model.nq} position variables (joints).")

            if visualize:
                # --- NEW: Initialize the Meshcat Visualizer ---
                logging.info("Starting Meshcat visualizer...")
                self.viz = MeshcatVisualizer(self.model, self.visual_model, self.visual_model)
                self.viz.initViewer(open=True)
                self.viz.loadViewerModel()
                logging.info("✅ Visualizer started. A browser tab should have opened.")

        except Exception as e:
            logging.error(f"❌ Failed to initialize KinematicsSolver: {e}")
            raise

    def get_frame_pose(self, frame_id, q_full):
        """Internal helper to get a pose from a full joint vector."""
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        frame_pose_se3 = self.data.oMf[frame_id]
        pose_xyzquat = pin.SE3ToXYZQUAT(frame_pose_se3) 
        return pose_xyzquat

    def get_all_poses(self, arm_joint_angles: np.ndarray, hand_joint_angles: np.ndarray = None):
        """
        Performs FK and returns the full joint vector and a dictionary of poses.
        """
        q_arm = np.asarray(arm_joint_angles)
        
        # --- THE CRITICAL FIX ---
        # 1. We receive the hand joints in "Raw LEAP" format (from the dataset)
        q_hand_raw = np.asarray(hand_joint_angles) if hand_joint_angles is not None else np.zeros(16)
        
        # 2. We must convert them back to "Allegro" format, which the URDF expects.
        #    (Assuming the inverse function exists in the library)
        #   Unscramble the hand joint orders accordingly (inverse of the teleop mapping)
        try:
            # Step A: Apply the inverse offset (e.g., subtract 3.14159)
            # This gives us the "qpos_cmd" format from the teleop script.
            qpos_cmd = lhu.LEAPhand_to_allegro(q_hand_raw, zeros=False)
            
            # Step B: Apply the inverse reordering to get the "Allegro" format.
            q_hand_allegro = np.zeros(16)
            q_hand_allegro[1] = qpos_cmd[0]  # Index rotation
            q_hand_allegro[0] = qpos_cmd[1]  # Index base
            q_hand_allegro[2] = qpos_cmd[2]  # Index middle
            q_hand_allegro[3] = qpos_cmd[3]  # Index tip

            q_hand_allegro[9] = qpos_cmd[4]  # Middle rotation
            q_hand_allegro[8] = qpos_cmd[5]  # Middle base
            q_hand_allegro[10] = qpos_cmd[6] # Middle middle
            q_hand_allegro[11] = qpos_cmd[7] # Middle tip

            q_hand_allegro[13] = qpos_cmd[8]  # Pinky rotation
            q_hand_allegro[12] = qpos_cmd[9]  # Pinky base
            q_hand_allegro[14] = qpos_cmd[10] # Pinky middle
            q_hand_allegro[15] = qpos_cmd[11] # Pinky tip

            q_hand_allegro[4] = qpos_cmd[12] # Thumb base
            q_hand_allegro[5] = qpos_cmd[13] # Thumb rotation
            q_hand_allegro[6] = qpos_cmd[14] # Thumb middle
            q_hand_allegro[7] = qpos_cmd[15] # Thumb tip

        except Exception as e:
            logging.error(f"Hand joint conversion failed: {e}")
            q_hand_allegro = np.zeros(16) # Fallback to a zero pose

        # 3. Create the full joint vector (q_full) using the converted "Allegro" angles
        q_full = np.zeros(self.model.nq) 
        q_full[:7] = q_arm
        if len(q_full[7:]) == len(q_hand_allegro):
             q_full[7:] = q_hand_allegro
        
        # 4. Now, Pinocchio receives the joint angles in the format it expects.
        poses = {}
        poses['ee'] = self.get_frame_pose(self.ee_frame_id, q_full)
        
        for name, frame_id in self.tactile_frame_ids.items():
            poses[name] = self.get_frame_pose(frame_id, q_full)
            
        return q_full, poses


    def get_ee_pose(self, arm_joint_angles: np.ndarray, hand_joint_angles: np.ndarray = None) -> np.ndarray:
        """Helper function to get the main end-effector pose."""
        q_full, poses = self.get_all_poses(arm_joint_angles, hand_joint_angles)
        return poses['ee']

    def get_all_tactile_poses(self, arm_joint_angles: np.ndarray, hand_joint_angles: np.ndarray = None) -> dict:
        """Helper function to get the poses of all tracked tactile sensors."""
        q_full, poses = self.get_all_poses(arm_joint_angles, hand_joint_angles)
        # Remove the 'ee' pose to only return tactile poses
        poses.pop('ee', None)
        return poses

    def update_visualizer(self, q_full: np.ndarray):
        """Updates the 3D model in the Meshcat browser."""
        self.viz.display(q_full)

