# leap_node.py
import numpy as np

from loguru import logger
from std_msgs.msg import Float64MultiArray

from dex_retargeting.leap_hand_utils.dynamixel_client import *
import dex_retargeting.leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import LeapPosition, LeapVelocity, LeapEffort, LeapPosVelEff

from rclpy.node import Node

from sensor_msgs.msg import JointState

class LeapNode(Node):
    def __init__(self):
        super().__init__('leaphand_node')
        ####Some parameters
        # self.ema_amount = float(rospy.get_param('/leaphand_node/ema', '1.0')) #take only current
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 550
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))

        self.create_service(LeapPosition, '/leap_position', self.pos_srv)
        self.get_logger().info("Leap service /leap_position created")
        # self.create_service(LeapVelocity, 'leap_velocity', self.vel_srv)
        # self.create_service(LeapEffort, 'leap_effort', self.eff_srv)
        # self.create_service(LeapPosVelEff, 'leap_pos_vel_eff', self.pos_vel_eff_srv)
        # self.create_service(LeapPosVelEff, 'leap_pos_vel', self.pos_vel_srv)
           
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
        # #Lower current limit for tip motors
        # self.dxl_client.sync_write([3,7,11,15], np.ones(4) * (self.curr_lim * 0.02), 102, 2)

        # pinch_motors = [1, 2, 3, 13, 14, 15] # Motors for pinch fingers
        # self.dxl_client.sync_write(pinch_motors, np.ones(len(pinch_motors)) * 1000, 84, 2)  # Higher P gain
        # self.dxl_client.sync_write(pinch_motors, np.ones(len(pinch_motors)) * 300, 80, 2) # Higher D gain
        # self.dxl_client.sync_write(pinch_motors, np.ones(len(pinch_motors)) * 1300, 102, 2)  # Higher current limit
        hand_control = True
        if hand_control:
            self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        else:
            logger.info("[WARNING] Hand control is disabled.")

        self.publisher_ = self.create_publisher(JointState, '/leap_hand_joint_cmd', 10)
        self.publisher2_ = self.create_publisher(JointState, '/leap_hand_joint_state', 10)
        self._joint_names = [f"leap_joint_{i}" for i in range(16)]

        # --- ADD a Subscriber to receive commands ---
        self.create_subscription(
            Float64MultiArray,
            '/leap_hand/target_allegro_pose',
            self.set_hand_position_callback,
            10)
        self.get_logger().info("✅ LEAP Hand Server is running. Providing '/leap_position' service and listening on '/leap_hand/target_allegro_pose'.")

    def set_hand_position_callback(self, msg):
        """Receives a command in Allegro format and sets the hand position."""
        target_pose = np.array(msg.data)
        if len(target_pose) == 16:
            # The 'set_leap' function from your teleop script handles all conversions
            self.set_leap(target_pose)
            self.get_logger().info("Received and executed new hand pose command.")
        else:
            self.get_logger().warn(f"Received invalid hand command with {len(target_pose)} joints.")

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
    # Service that reads and returns the pos of the robot in regular LEAP Embodiment scaling.
    def pos_srv(self, request, response):
        # self.get_logger().info("Received /leap_position request")
        response.position = self.dxl_client.read_pos().tolist()
        return response
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()
    def cleanup(self):
        # Send relax command to all motors
        try:
            self.dxl_client.set_torque_enabled(self.motors, False)
            print("[LEAP] Motors relaxed.")
        except Exception as e:
            print(f"[LEAP] Failed to relax motors: {e}")
