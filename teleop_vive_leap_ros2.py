import multiprocessing
import asyncio
import json
import open3d as o3d
import numpy as np
import cv2
import time

import tyro
from loguru import logger

from pathlib import Path
from queue import Empty

from dex_retargeting.constants import RobotName, RetargetingType, HandType, get_default_config_path
from dex_retargeting.retargeting_config import RetargetingConfig
from single_hand_detector import SingleHandDetector

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped

from scipy.spatial.transform import Rotation as R

from std_srvs.srv import Trigger

# --- Configuration and Initialization ---
target_rate = 30  # Target loop rate in Hz

# Open3D visualization setup
visualize=True  # Set to True to enable Open3D visualization
if visualize:
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    coordinate_frames = []

# Define a color map for tracker IDs
tracker_color_map = {
    "left_elbow": [1.0, 0.0, 0.0],  # Red
    "right_elbow": [0.0, 1.0, 0.0],  # Green
    "chest": [0.0, 0.0, 1.0],  # Blue
}

def get_color_for_tracker(tracker_id):
    """
    Get color for a tracker ID.
    """
    return tracker_color_map.get(tracker_id, [0.5, 0.5, 0.5])  # Default to gray if ID is unknown

def update_coordinate_frames(poses):
    """
    Update the coordinate frames in the Open3D visualizer based on the latest pose data.
    """
    start_time = time.perf_counter()

    # Clear all existing geometries
    vis.clear_geometries()

    # Add new geometries
    global coordinate_frames
    coordinate_frames = []
    for pose in poses:
        position = pose["position"]
        orientation = pose["orientation"]
        tracker_id = pose.get("tracker_id", "unknown_tracker")

        pos = np.array([position["x"], position["y"], position["z"]])
        q = [orientation["w"], orientation["x"], orientation["y"], orientation["z"]]

        # Get color for the tracker ID
        color = get_color_for_tracker(tracker_id)

        # Create and position the coordinate frame
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        frame.translate(pos)
        frame.rotate(o3d.geometry.get_rotation_matrix_from_quaternion(q), center=pos)

        # Apply color to the axes
        for triangle in frame.triangles:
            frame.paint_uniform_color(color)

        vis.add_geometry(frame)
        coordinate_frames.append(frame)

    # Force Open3D to refresh the scene
    vis.poll_events()
    vis.update_renderer()

    end_time = time.perf_counter()
    # print(f"[Timing] update_coordinate_frames: {end_time - start_time:.6f} seconds")

# --- VIVE UDP Receiver ---
class UDPReceiverProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.poses = []

    def datagram_received(self, data, addr):
        try:
            pose_data = json.loads(data.decode('utf-8'))
            self.poses = pose_data.get("poses", [])
        except Exception as e:
            print(f"Error processing data: {e}")

async def render_loop(protocol):
    """
    Periodically update the Open3D visualization with the latest pose data.
    """
    while True:
        start_time = time.perf_counter()

        # Update visualization only if there are poses
        if protocol.poses and visualize:
            update_coordinate_frames(protocol.poses)

        end_time = time.perf_counter()
        # print(f"[Timing] render_loop: {end_time - start_time:.6f} seconds")
        # await asyncio.sleep(0.01)  # 30 FPS            

class LeapNode:
    def __init__(self):
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 550
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
           
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM13', 4000000)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position
    def read_pos(self):
        return self.dxl_client.read_pos()
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()

def start_retargeting(queue: multiprocessing.Queue, robot_dir: str, config_path: str, leaphand :LeapNode):
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    logger.info(f"Start retargeting with config {config_path}")
    retargeting = RetargetingConfig.load_from_file(config_path).build()

    hand_type = "Right" if "right" in config_path.lower() else "Left"
    detector = SingleHandDetector(hand_type=hand_type, selfie=False)


    # Different robot loader may have different orders for joints
    # sapien_joint_names = [joint.get_name() for joint in robot.get_active_joints()]
    # retargeting_joint_names = retargeting.joint_names
    # retargeting_to_sapien = np.array([retargeting_joint_names.index(name) for name in sapien_joint_names]).astype(int)

    while True:
        start_t = time.time()
        try:
            rgb = queue.get(timeout=50)
        except Empty:
            logger.error(f"Fail to fetch image from camera in 50 secs. Please check your web camera device.")
            return

        _, joint_pos, _, _ = detector.detect(rgb)
        if joint_pos is None:
            # logger.warning(f"{hand_type} hand is not detected.")
            pass
        else:
            retargeting_type = retargeting.optimizer.retargeting_type
            indices = retargeting.optimizer.target_link_human_indices
            if retargeting_type == "POSITION":
                indices = indices
                ref_value = joint_pos[indices, :]
            else:
                origin_indices = indices[0, :]
                task_indices = indices[1, :]
                ref_value = joint_pos[task_indices, :] - joint_pos[origin_indices, :]
            qpos = retargeting.retarget(ref_value)
            # print("qpos: " + ", ".join(f"{pos:.4f}" for pos in qpos))

            qpos_cmd = np.zeros(16)
            # current_pos = leaphand.read_pos()
            # diff = (qpos[0]- current_pos[0] + 3.14) 
            # if  abs(diff)> 0.001:
            #     diff = np.sign(diff) * 0.001
                 
            # qpos_cmd[0] =  current_pos[0] +  diff 

            # qpos_cmd[0] = qpos[0]
            # qpos_cmd[1] = qpos[1]
            # qpos_cmd[2] = qpos[2]
            # qpos_cmd[3] = qpos[3]

            # qpos_cmd[4] = qpos[8] # thumb - middle
            # qpos_cmd[5] = qpos[9]
            # qpos_cmd[6] = qpos[10]
            # qpos_cmd[7] = qpos[11]

            # qpos_cmd[8] = qpos[12] # none
            # qpos_cmd[9] = qpos[13]
            # qpos_cmd[10] = qpos[14]
            # qpos_cmd[11] = qpos[15]

            # qpos_cmd[12] = qpos[4] # thumb - middle 
            # qpos_cmd[13] = qpos[5]
            # qpos_cmd[14] = qpos[6]
            # qpos_cmd[15] = qpos[7]

            # ['1', '0', '2', '3', '12', '13', '14', '15', '5', '4', '6', '7', '9', '8', '10', '11']

            qpos_cmd[0] = qpos[1]
            qpos_cmd[1] = qpos[0]
            qpos_cmd[2] = qpos[2]
            qpos_cmd[3] = qpos[3]

            qpos_cmd[4] = qpos[9] # thumb - middle
            qpos_cmd[5] = qpos[8]
            qpos_cmd[6] = qpos[10]
            qpos_cmd[7] = qpos[11]

            qpos_cmd[8] = qpos[13] # none
            qpos_cmd[9] = qpos[12]
            qpos_cmd[10] = qpos[14]
            qpos_cmd[11] = qpos[15]

            qpos_cmd[12] = qpos[4] # thumb - middle 
            qpos_cmd[13] = qpos[5]
            qpos_cmd[14] = qpos[6]
            qpos_cmd[15] = qpos[7]

            # qpos_cmd[8] = qpos[8]        

            # qpos_cmd = qpos
            # print(f"{qpos_cmd[1]:.4f}")
            # print("qpos_cmd: " + ", ".join(f"{pos:.4f}" for pos in qpos_cmd))
            end_t = time.time()
            # print(f"time: {end_t - start_t:.4f} s")

            leaphand.set_allegro(qpos_cmd)


            # print("Position: " + str(leaphand.read_pos()))
            # time.sleep(0.02)
            # a = input("test")


# --- Camera Producer (runs in a process) ---
def produce_frame(queue, camera_path=None):
    cap = cv2.VideoCapture(camera_path or '/dev/video0')
    if not cap.isOpened():
        print(f"[ERROR] Failed to open camera: {camera_path}")
        return
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            continue
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        queue.put(image_rgb)
        cv2.imshow("Camera", image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

def produce_realsense_frame(queue: multiprocessing.Queue):
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  

    try:
        pipeline.start(config)
        print("[INFO] RealSense D405 camera started.")
    except Exception as e:
        print(f"[ERROR] Failed to start RealSense pipeline: {e}")
        return

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # 轉為 numpy array
            color_image = np.asanyarray(color_frame.get_data())
            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            queue.put(rgb_image)
            cv2.imshow("RealSense", color_image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] RealSense pipeline stopped.")

class ViveToROS2Publisher(Node):
    def __init__(self):
        super().__init__('vive_to_ros2_publisher')
        self.twist_publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.relative_publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.publisher_ = self.create_publisher(PoseStamped, '/vive_pose', 10)
        self.alpha = 0.2  # Smoothing factor for EMA
        self._filter_state = {}  # Store filtered values for each key

        self.prev_tracker_pos = None    # Previous tracker position for velocity calculation
        self.prev_tracker_ori = None    # Previous tracker orientation for velocity calculation
        self.prev_time = None   # Previous time for velocity calculation

        self.robot_current_pose = None  # Current pose of the robot, updated from the robot state broadcaster
        self.robot_initial_pose = None  # Initial pose of the robot, updated from the robot state broadcaster
        self.tracker_initial_pos = None # Initial position of the tracker, set when resetting origin
        self.tracker_initial_ori = None # Initial orientation of the tracker, set when resetting origin
        self.current_tracker_pos = None # Current position of the tracker, updated from VIVE data
        self.current_tracker_ori = None # Current orientation of the tracker, updated from VIVE data
        self.tracking_enabled = False    # Flag to enable/disable tracking
        self.robot_pose_subscription = self.create_subscription(    
            PoseStamped, '/franka_robot_state_broadcaster/current_pose', self.robot_pose_callback, 1)   # Subscribe to robot pose updates
        self.srv = self.create_service(Trigger, '/reset_tracking_origin', self.reset_origin_callback)    # Service to reset tracking origin
        self.alpha = 0.2  # Smoothing factor for velocity

    def robot_pose_callback(self, msg):
        self.robot_current_pose = msg  # Always update, so we have the latest

    def reset_origin_callback(self, request, response):
        if self.current_tracker_pos is not None and self.current_tracker_ori is not None and self.robot_current_pose is not None:
            self.tracker_initial_pos = self.current_tracker_pos
            self.tracker_initial_ori = self.current_tracker_ori
            self.robot_initial_pose = self.robot_current_pose  # Use the latest robot pose as the initial pose
            # robot_initial_pose is already up-to-date from callback
            self.tracking_enabled = True
            response.success = True
            response.message = "Tracking origin reset and tracking enabled."
        else:
            print("current_tracker_pos:", self.current_tracker_pos)
            print("current_tracker_ori:", self.current_tracker_ori)
            print("robot_current_pose:", self.robot_current_pose)
            response.success = False
            response.message = "Tracker or robot pose not available."
        return response
    
    def publish_relative_pose(self, tracker_pos, tracker_ori):
        # Wait until both initial poses are set
        if self.robot_initial_pose is None or self.tracker_initial_pos is None:
            return

        # --- Filtering ---
        tracker_pos = self.filter_value(tracker_pos, key="rel_pos")
        tracker_ori = self.filter_value(tracker_ori, key="rel_ori", is_orientation=True)

        # Compute tracker delta (current - initial)
        delta_pos = tracker_pos - self.tracker_initial_pos
        delta_ori = R.from_quat(tracker_ori) * R.from_quat(self.tracker_initial_ori).inv()

        # Apply delta to robot initial pose
        robot_init_pos = np.array([
            self.robot_initial_pose.pose.position.x,
            self.robot_initial_pose.pose.position.y,
            self.robot_initial_pose.pose.position.z
        ])
        robot_init_ori = [
            self.robot_initial_pose.pose.orientation.x,
            self.robot_initial_pose.pose.orientation.y,
            self.robot_initial_pose.pose.orientation.z,
            self.robot_initial_pose.pose.orientation.w
        ]

        new_pos = robot_init_pos + delta_pos
        new_ori = (delta_ori * R.from_quat(robot_init_ori)).as_quat()  # [x, y, z, w]

        # Publish
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.pose.position.x = new_pos[0]
        msg.pose.position.y = new_pos[1]
        msg.pose.position.z = new_pos[2]
        msg.pose.orientation.x = new_ori[0]
        msg.pose.orientation.y = new_ori[1]
        msg.pose.orientation.z = new_ori[2]
        msg.pose.orientation.w = new_ori[3]
        self.relative_publisher.publish(msg)

    def publish_pose(self, position, orientation):
        # # Filter position and orientation
        # position = self.filter_value(position, key="pose_pos")
        # orientation = self.filter_value(orientation, key="pose_ori", is_orientation=True)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3]
        self.publisher_.publish(msg)

    def publish_twist(self, current_pos, current_ori):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_tracker_pos is None or self.prev_tracker_ori is None or self.prev_time is None:
            self.prev_tracker_pos = current_pos
            self.prev_tracker_ori = current_ori
            self.prev_time = now
            print("Initializing previous tracker position and orientation.")
            return

        dt = now - self.prev_time
        if dt <= 0.005 or dt > 0.2:
            print(f"Skipping update: dt={dt:.4f} too small or too large")
            return

        # Filter position and orientation
        current_pos = self.filter_value(current_pos, key="twist_pos")
        current_ori = self.filter_value(current_ori, key="twist_ori", is_orientation=True)

        linear = (current_pos - self.prev_tracker_pos) / dt

        r_prev = R.from_quat(self.prev_tracker_ori)
        r_curr = R.from_quat(current_ori)
        r_rel = r_curr * r_prev.inv()
        rotvec = r_rel.as_rotvec()
        angular = rotvec / dt

        max_lin = 2.0
        max_ang = 6.0
        linear = np.clip(linear, -max_lin, max_lin)
        angular = np.clip(angular, -max_ang, max_ang)

        if hasattr(self, "prev_linear"):
            linear = self.alpha * linear + (1 - self.alpha) * self.prev_linear
            angular = self.alpha * angular + (1 - self.alpha) * self.prev_angular

        self.prev_linear = linear
        self.prev_angular = angular

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
        self.twist_publisher.publish(msg)

        self.prev_tracker_pos = current_pos
        self.prev_tracker_ori = current_ori
        self.prev_time = now

    def smooth(self, new, old):
        return self.alpha * new + (1 - self.alpha) * old

    def filter_value(self, new, key, is_orientation=False):
        """
        Exponential Moving Average filter for positions or orientations.
        Stores previous filtered values in self._filter_state[key].
        If is_orientation is True, uses SLERP for quaternion smoothing.
        """
        if key not in self._filter_state:
            self._filter_state[key] = new
            return new

        if is_orientation:
            from scipy.spatial.transform import Slerp
            times = [0, 1]
            key_rots = R.from_quat([self._filter_state[key], new])
            slerp = Slerp(times, key_rots)
            filtered = slerp(self.alpha).as_quat()
        else:
            filtered = self.alpha * new + (1 - self.alpha) * self._filter_state[key]

        self._filter_state[key] = filtered
        return filtered

# --- Main Async Loop ---
async def main(args=None):
    # UDP setup
    UDP_IP = "0.0.0.0"
    UDP_PORT = 5005
    loop = asyncio.get_running_loop()
    vive_protocol = UDPReceiverProtocol()
    await loop.create_datagram_endpoint(
        lambda: vive_protocol,
        local_addr=(UDP_IP, UDP_PORT),
    )

    # ROS2 setup
    rclpy.init(args=args)
    node = ViveToROS2Publisher()

    # Camera setup (multiprocessing)
    queue = multiprocessing.Queue(maxsize=1)
    camera_path = None  # or set to your camera device

    cap = cv2.VideoCapture(camera_path or 0)
    if cap.isOpened():
        producer_process = multiprocessing.Process(target=produce_frame, args=(queue, camera_path))
    else:
        print("[INFO] Webcam not found, trying RealSense D405.")
        producer_process = multiprocessing.Process(target=produce_realsense_frame, args=(queue,))
    producer_process.start()

    # LEAP Hand setup
    leap_hand = LeapNode()
    config_path = Path(__file__).resolve().parents[2] / "dex_retargeting/configs/teleop/leap_hand_right_dexpilot.yml"
    robot_dir = Path(__file__).resolve().parents[2] / "assets/robots/hands"
    consumer_process = multiprocessing.Process(target=start_retargeting, args=(queue, str(robot_dir), str(config_path), leap_hand))
    consumer_process.start()


    loop_counter = 0
    print_interval = 100  # Print every 100 loops
    last_print_time = time.time()
    target_dt = 1.0 / target_rate

    try:
        while rclpy.ok():
            loop_start = time.time()
            loop_counter += 1
            
            # --- Process Vive tracker data ---
            vive_poses = vive_protocol.poses
            # print(f"[DEBUG] Received {len(vive_poses)} VIVE poses", flush=True)
            if vive_poses:
                # Example: Use the third tracker as the end-effector pose
                pose = vive_poses[0]
                # Convert position from VIVE to ROS (MoveIt) coordinates
                # VIVE: x (right), y (up), z (backward)
                # ROS:  x (forward), y (left), z (up)

                position = np.array([
                    -pose["position"]["z"],  # ROS X = -VIVE Z (forward/backward)
                    -pose["position"]["x"],  # ROS Y = -VIVE X (left/right)
                    pose["position"]["y"]    # ROS Z = VIVE Y (up/down)
                ])

                # Convert orientation (quaternion) from VIVE to ROS
                # VIVE quaternion: [w, x, y, z]
                orientation = np.array([
                    -pose["orientation"]["z"],
                    -pose["orientation"]["x"],
                    pose["orientation"]["y"],
                    pose["orientation"]["w"]
                ])

                # # Define the rotation from VIVE to ROS axes
                # # Example: 90 deg about Y then 90 deg about Z (adjust as needed)
                # # This is a common mapping for VIVE to ROS, but test for your setup!
                # R_vive_to_ros = R.from_euler('zyx', [0,0,0])
                # R_vive = R.from_quat(q_vive)  # [x, y, z, w]
                # R_ros = R_vive_to_ros * R_vive
                # q_ros = R_ros.as_quat()  # [x, y, z, w]

                # orientation = np.array([
                #     q_ros[0],  # x
                #     q_ros[1],  # y
                #     q_ros[2],  # z
                #     q_ros[3],  # w
                # ])

                # print(f"VIVE Pose: {pose}")
                # print(f"ROS Position: {position}")
                # print(f"ROS Orientation: {orientation}")

                node.current_tracker_pos = position
                node.current_tracker_ori = orientation
                node.publish_pose(position, orientation)
                if node.tracking_enabled:
                    node.publish_relative_pose(position, orientation)
                # node.publish_twist(position, orientation)
            rclpy.spin_once(node, timeout_sec=0.01)

            # --- Visualization ---
            if visualize:
                if vive_poses:
                    update_coordinate_frames(vive_poses)
                # await asyncio.sleep(0.01)  # 30 FPS for visualization

            # --- Print actual loop rate ---
            if loop_counter % print_interval == 0:
                now = time.time()
                elapsed = now - last_print_time
                rate = print_interval / elapsed if elapsed > 0 else 0
                print(f"[INFO] Actual publishing rate: {rate:.2f} Hz over last {print_interval} loops", flush=True)
                last_print_time = now

            # --- Sleep only for the remaining time to maintain target rate ---
            loop_end = time.time()
            elapsed_loop = loop_end - loop_start
            sleep_time = max(0, target_dt - elapsed_loop)
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                print(f"[WARNING] Loop took too long: {elapsed_loop:.4f} seconds, skipping sleep", flush=True)

    finally:
        producer_process.terminate()
        consumer_process.terminate()
        cap.release()
        cv2.destroyAllWindows()
        if visualize:
            vis.destroy_window()
        rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())