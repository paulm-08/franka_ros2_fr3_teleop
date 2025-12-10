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
import json
from geometry_msgs.msg import PoseStamped 
from moveit_msgs.srv import GetPositionFK 

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

def get_control_mode(model_path_str):
    """
    Infers the control mode (joint or cartesian) from the model's checkpoint config.
    Assumes config is stored in a JSON file named 'config.json' next to the checkpoint.
    """
    model_dir = Path(model_path_str).parent
    config_path = model_dir / "config.json"
    
    if not config_path.exists():
        logging.warning(f"Config file not found at {config_path}. Defaulting to 'joint'.")
        return "joint"

    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        # Check a specific key that defines the control type
        control_type = config.get('control_type', 'joint').lower()
        if 'cartesian' in control_type or 'servo' in control_type:
            return "cartesian"
        return "joint"

    except Exception as e:
        logging.error(f"Error reading model config: {e}. Defaulting to 'joint'.")
        return "joint"

class StagingController(Node):
    """A single node to handle both arm and hand staging commands."""
  
    def __init__(self, control_mode): # <-- Take control_mode argument
        super().__init__('staging_controller_node')
        self.control_mode = control_mode
        
        # --- Arm Controller Interface ---
        if self.control_mode == 'cartesian':
            # Cartesian Interface: Publishes PoseStamped to Servo
            self.arm_publisher = self.create_publisher(PoseStamped, '/servo_node/pose_tracking_pose', 10)
            
            # FK Client for Cartesian staging
            self.fk_client = self.create_client(GetPositionFK, 'compute_fk')
            while not self.fk_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('FK service not available, waiting...')

        else:
            # Joint Interface: Publishes JointTrajectory
            self.arm_publisher = self.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10)
        
        # --- Hand Controller Interface ---
        self.hand_publisher = self.create_publisher(Float64MultiArray, '/leap_hand/target_allegro_pose', 10)

    def _get_cartesian_pose_from_joint(self, joint_pose):
        """Uses MoveIt's FK service to convert a joint pose to an end-effector pose."""
        if not self.fk_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().error("FK Service unavailable.")
            return None

        # Create the request
        req = GetPositionFK.Request()
        req.header.frame_id = 'base_link'
        req.fk_link_names = ['leap_hand_link'] # The end-effector link name
        
        # Set the joint positions
        req.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        req.robot_state.joint_state.name = [f'fr3_joint{i+1}' for i in range(7)]
        req.robot_state.joint_state.position = [float(p) for p in joint_pose]
        
        # Call the service
        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().error_code.val == future.result().error_code.SUCCESS:
            # Result contains the PoseStamped
            return future.result().pose_stamped[0].pose
        else:
            self.get_logger().error(f"FK calculation failed: {future.result().error_code.val}")
            return None


    def command_arm(self, target_pose_list, duration_sec):
        """
        Commands the arm. target_pose_list is ALWAYS the 7-DOF Joint Pose q_start.
        """
        if self.control_mode == 'joint':
            # JOINT CONTROL MODE: Publish JointTrajectory directly
            msg = JointTrajectory()
            msg.joint_names = [f'fr3_joint{i+1}' for i in range(7)]
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in target_pose_list]
            point.time_from_start = Duration(sec=int(duration_sec))
            msg.points.append(point)
            
            self.arm_publisher.publish(msg)
            self.get_logger().info(f"Arm move command (Joint Trajectory) sent. Waiting {duration_sec}s...")

        elif self.control_mode == 'cartesian':
            # CARTESIAN CONTROL MODE: 
            # 1. Convert Joint Pose to Cartesian Pose using FK
            target_pose = self._get_cartesian_pose_from_joint(target_pose_list)
            
            if target_pose is None:
                self.get_logger().error("Cannot stage arm: FK failed.")
                return

            # 2. Publish PoseStamped to Servo node
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link' 
            msg.pose = target_pose # Pose geometry message
            
            self.arm_publisher.publish(msg)
            self.get_logger().info(f"Arm move command (Cartesian/FK) sent. Waiting {duration_sec}s...")

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

        # 1. Determine Control Mode (using the function above)
        control_mode = get_control_mode(selected_model)
        logging.info(f"Selected model requires **{control_mode.upper()}** control.")

        # 2. Configure launch arguments for bringup.launch.py
        bringup_args = ["ros2", "launch", "franka_fr3_moveit_config", "bringup.launch.py"]
        
        if control_mode == "cartesian":
            # Pass the new YAML that only contains joint_state_broadcaster
            bringup_args.append("controllers_yaml_file:=fr3_ros_controllers_servo.yaml")
            # Ensure the pose_tracking node is enabled in bringup.launch.py (by un-commenting it)
            # Assuming you've already made the changes from the previous answer.
        else:
            # Default joint control (fr3_arm_controller)
            bringup_args.append("controllers_yaml_file:=fr3_ros_controllers_rollout.yaml")
            # Assuming fr3_ros_controllers_rollout.yaml contains fr3_arm_controller

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