import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import time

class RefreshTester(Node):
    def __init__(self):
        super().__init__('refresh_tester')

        # --- CONFIGURATION ---
        self.PUBLISH_RATE_HZ = 2.0  # Must be fast (e.g., 20 Hz, matching your control_rate or faster)
        self.TIME_FROM_START_SEC = 0.05 # Controller target duration (50 ms)
        
        # Canonical joint names (controller's expected order)
        self.canonical_joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]

        # --- STATE MANAGEMENT ---
        self.current_arm_states = None
        self.received_state_order = None 
        self.state_to_policy_indices = None 

        # --- ROS SETUP ---
        # Subscriber for robot's actual joint positions
        self.state_sub = self.create_subscription(
            JointState,
            '/franka/joint_states', # Check your actual state topic name
            self.joint_state_callback,
            10
        )
        
        # Publisher for the robot's command
        self.command_pub = self.create_publisher(
            JointTrajectory,
            '/fr3_arm_controller/joint_trajectory', # Check your actual command topic name
            10
        )
        
        # Main publishing timer (mimics your control loop's rate)
        self.timer = self.create_timer(1.0 / self.PUBLISH_RATE_HZ, self.publish_refresh_command)
        self.get_logger().info("Refresh Tester Node Initialized. Awaiting first joint state...")


    def joint_state_callback(self, msg: JointState):
        """Processes incoming joint states and re-orders them to the canonical sequence."""
        if len(msg.position) < 7:
            return

        # Dynamic Mapping Check (Run only once)
        if self.state_to_policy_indices is None:
            self.received_state_order = msg.name[:7]
            
            try:
                # Find the index of each canonical joint within the received message's name list
                self.state_to_policy_indices = np.array([
                    self.received_state_order.index(name) 
                    for name in self.canonical_joint_names
                ])
                self.get_logger().info("✅ Dynamic mapping successful.")
            except ValueError:
                self.get_logger().error(
                    f"Joint name mismatch! Expected joints not found in received list: {self.received_state_order}"
                )
                self.destroy_node() # Crash if names are wrong

        # Re-order the data to the CANONICAL (J1-J7) order
        arm_positions_received_order = np.array(msg.position[:7])
        self.current_arm_states = arm_positions_received_order[self.state_to_policy_indices]


    def publish_refresh_command(self):
        """Publishes the current position as the next target (Delta Q = 0)."""
        if self.current_arm_states is None:
            return

        # The target position is simply the current position (Delta Q = 0)
        target_q_arm = self.current_arm_states
        
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.canonical_joint_names 
        
        point = JointTrajectoryPoint()
        point.positions = target_q_arm.tolist()
        
        # Set a short time-from-start duration
        duration_sec = int(self.TIME_FROM_START_SEC)
        duration_nanosec = int((self.TIME_FROM_START_SEC - duration_sec) * 1e9)
        point.time_from_start = Duration(sec=duration_sec, nanosec=duration_nanosec)
        
        traj_msg.points.append(point)
        self.command_pub.publish(traj_msg)
        
        self.get_logger().debug(f"Published target: {target_q_arm[:3]}...")


def main(args=None):
    rclpy.init(args=args)
    tester_node = RefreshTester()
    
    # Wait for the first state message to initialize current_arm_states
    tester_node.get_logger().info("Waiting for joint state to initialize...")
    start_time = time.time()
    while rclpy.ok() and tester_node.current_arm_states is None and (time.time() - start_time < 5):
        rclpy.spin_once(tester_node, timeout_sec=0.1)

    if tester_node.current_arm_states is not None:
        tester_node.get_logger().info("Starting constant refresh loop.")
        try:
            rclpy.spin(tester_node)
        except KeyboardInterrupt:
            pass
    else:
        tester_node.get_logger().error("Failed to receive initial joint state. Exiting.")

    tester_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()