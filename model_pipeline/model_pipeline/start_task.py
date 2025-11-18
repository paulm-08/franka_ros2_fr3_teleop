import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import subprocess
import yaml
import inquirer
import logging
import numpy as np
from std_msgs.msg import Float64MultiArray
import threading
from rclpy.executors import MultiThreadedExecutor

# --- Direct Hardware/Control Imports (from your teleop script) ---
from dex_retargeting.leap_hand_utils.dynamixel_client import DynamixelClient
import dex_retargeting.leap_hand_utils.leap_hand_utils as lhu

from model_pipeline import paths
from model_pipeline.leap_node import LeapNode
from model_pipeline.evaluate_policy import find_policy_models


logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def find_goal_files(search_path):
    goal_files = sorted([p.relative_to(paths.WORKSPACE_ROOT) for p in search_path.glob("*goal*.pkl")])
    return goal_files

class StagingController(Node):
    """A single node to handle both arm and hand staging commands."""
    def __init__(self):
        super().__init__('staging_controller_node')
        
        # --- Arm Controller Interface ---
        self.arm_publisher = self.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10)
        
        # --- Hand Controller Interface (Mirrors your LeapNode) ---
        self.hand_publisher = self.create_publisher(Float64MultiArray, '/leap_hand/target_allegro_pose', 10)

    def command_arm(self, target_pose, duration_sec):
        msg = JointTrajectory()
        msg.joint_names = [f'fr3_joint{i+1}' for i in range(7)]
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in target_pose]
        point.time_from_start = Duration(sec=int(duration_sec))
        msg.points.append(point)
        
        self.arm_publisher.publish(msg)
        self.get_logger().info(f"Arm move command sent. Waiting {duration_sec}s...")

    def command_hand(self, leap_pose):
        """
        Commands the hand using a pose that is ALREADY in the raw LEAP Hand format.
        This performs NO conversion or reordering.
        """
        msg = Float64MultiArray()
        msg.data = [float(p) for p in leap_pose]
        self.hand_publisher.publish(msg)
        # self.get_logger().info("Hand command sent.")

def main():
    try:
        # --- 1. Interactively Select Model and Goal ---
        model_choices = find_policy_models(paths.POLICY_MODELS_DIR)
        goal_choices = find_goal_files(paths.PROCESSED_DATA_DIR)

        if not model_choices or not goal_choices:
            logging.error("No models or goal files found. Please train a model and create a goal state first.")
            return

        questions = [
            inquirer.List('model_path', message="Select the policy model to deploy", choices=model_choices),
            inquirer.List('goal_state_path', message="Select the goal state for this task", choices=goal_choices),
        ]
        answers = inquirer.prompt(questions)
        if not answers:
            logging.info("No selection made. Exiting.")
            return
            
        selected_model = str(answers['model_path'])
        selected_goal = str(answers['goal_state_path'])

        # --- Launch Robot Drivers ---
        logging.info("Launching robot drivers...")
        bringup_process = subprocess.Popen(
            ["ros2", "launch", "franka_fr3_moveit_config", "bringup.launch.py"],
        )
        logging.info("Waiting 5s for drivers to initialize...")
        time.sleep(5) # Give drivers time to come up

        # --- Execute Staging Sequence (Grasp & Move) ---
        rclpy.init()
        leaphand = LeapNode()
        executor = MultiThreadedExecutor()
        executor.add_node(leaphand)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # --- 2. Load Setup Poses from Config ---
        with open(paths.DEFAULT_CONFIG_PATH, 'r') as f:
            config = yaml.safe_load(f).get('robot_setup', {})
        start_pose_arm = config.get('start_pose_arm')
        hand_grasp_pose = config.get('hand_grasp_pose')
        
        # --- 3. Execute Staging Sequence (Grasp & Move) ---
        staging_node = StagingController()
        
        open_hand_pose = lhu.allegro_to_LEAPhand([0.0]*16, zeros=False)
        staging_node.command_hand(open_hand_pose)  # Open hand
        time.sleep(1) # Wait for hand open

        staging_node.command_arm(start_pose_arm, duration_sec=3)
        time.sleep(3) # Wait for arm move

        # # staging_node.command_hand(hand_grasp_pose)
        # # time.sleep(3) # Wait for grasp
        # logging.info("Commanding a slow, interpolated grasp...")

        # # Define grasp parameters
        # grasp_duration = 2.0  # Total time for the grasp in seconds
        # num_steps = 100        # Number of intermediate steps
        # hand_grasp_pose = np.array(hand_grasp_pose)

        # # Loop to send intermediate poses
        # for i in range(num_steps - 5):
        #     # Calculate the interpolation factor (alpha) from 0.0 to 1.0
        #     alpha = i / num_steps
            
        #     # Linearly interpolate between the open and closed poses
        #     intermediate_pose = (1 - alpha) * open_hand_pose + alpha * hand_grasp_pose
            
        #     # Send the command (convert back to list for the function)
        #     staging_node.command_hand(intermediate_pose.tolist())

        #     # Wait for a short duration before the next step
        #     time.sleep(grasp_duration / num_steps)
        
        # logging.info("Grasp complete.")
        # time.sleep(1) # A final short pause after grasping
           
        # --- Launch the Autonomous Policy Node ---
        logging.info("\n✅ Robot is at start pose. Launching policy node...")
                   
        command = [
            "ros2", "launch", "franka_fr3_moveit_config", "rollout.launch.py",
            f"model_path:={selected_model}",
            f"goal_state_path:={selected_goal}"
        ]
        
        subprocess.run(" ".join(command), shell=True, check=True)
        

    except (KeyboardInterrupt):
        logging.info("\nOperation cancelled by user.")
    except Exception as e:
        logging.error(f"An error occurred: {e}", exc_info=True)
    
    finally:
        # staging_node.cleanup()
        staging_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()