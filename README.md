# Franka FR3 Teleoperation Setup with LEAP Hand and Multimodal Feedback

<!-- ![Franka FR3 + LEAP Hand Setup](docs/images/setup_overview.png) Optional: replace with a real image if available -->

---

## 📌 Overview
This repository contains a complete setup for **teleoperating a Franka FR3 robotic arm** combined with a **LEAP Hand**.  
It integrates **motion tracking, hand retargeting, tactile sensing, visual feedback, and haptic feedback** to create a multimodal teleoperation system.

Note: The repository extends the official [`franka_ros2`](https://github.com/frankarobotics/franka_ros2) stack and integrates several additional components to enable real-time teleoperation. The original README.md was moved to the franka_ros2 package.

---

## ✨ Features
- **Low-level FR3 control** using [`libfranka`](https://frankarobotics.github.io/docs/libfranka.html) and ROS 2 controllers.
- **Real-time Cartesian control** via MoveIt Servo.
- **Wrist-mounted teleoperation device** using:
  - HTC Vive Ultimate Tracker for 6-DoF wrist tracking.
  - A camera-based haptic feedback system.
- **Hand pose tracking and retargeting** from human motion to the LEAP Hand via [DexRetargeting](https://github.com/dexsuite/dex-retargeting).
- **Visual feedback integration** using an RGB-D camera and ArUco-based hand-eye calibration via [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2).
- **Tactile feedback integration** using the [9DTact sensor](https://github.com/linchangyi1/9DTact).
- **Multimodal data collection** for imitation learning or reinforcement learning.
- Modular packages for **teleoperation**, **calibration**, **data recording**, and **retargeting**.

## 🛠️ Installation

### 1. Clone the repository

```bash
git clone https://github.com/paulm-08/franka_ros2_fr3_teleop.git
````

### 2. Install dependencies

Make sure you have ROS 2 (Humble or Iron) installed.
Then, install required ROS 2 dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

> **Tip:** If `diagnostic_updater` or other missing dependencies cause errors, install them manually:

```bash
sudo apt install ros-${ROS_DISTRO}-diagnostic-updater
```

### 3. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage

<!-- ### 1. Launch the teleoperation system -->

Launch the teleoperation system:
```bash
ros2 launch franka_fr3_moveit_config moveit_pose_cam.launch.py
```

This will:

* Stream wrist poses from the HTC Vive Ultimate tracker.
* Retarget human hand poses to the LEAP Hand.
* Control the Franka FR3 in real time.
* Record data from the robot pose, camera images and tactile sensor images
* Enable haptic feedback

```bash
ros2 run fr3_leap_teleop teleop_vive_leap_ros2.py
```

This will only:
* Stream wrist poses from the HTC Vive Ultimate tracker.
* Retarget human hand poses to the LEAP Hand.

<!-- ### 2. Visual feedback

With the [Realsense ROS2 wrapper](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file) installed, start a RealSense camera node:
```bash
ros2 launch realsense2_camera rs_launch.py
```
This publishes image and depth data as well as camera intrinsic parameters to the respective topics:
/camera/camera/color/camera_info
/camera/camera/color/image_raw,
/camera/camera/depth/image_raw


Start Aruco marker detector:
```bash
ros2 run easy_handeye2 aruco_tf_publisher --ros-args \
  -p marker_id:=0 \
  -p marker_size:=0.10 \
  -p camera_frame:=camera_link \
  -p tracking_marker_frame:=aruco_marker_frame
```
This publishes the pose of the marker relative to the camera as a TF transform (e.g., camera_link → aruco_marker).

```bash
ros2 launch easy_handeye2 aruco_calibration.launch.py
```

### 3. Tactile feedback

```bash
ros2 launch tact9d tact_sensor.launch.py
```

### 4. Data collection

```bash
ros2 run fr3_leap_recorder fr3_leap_recorder.py
``` -->

---

## 📦 Included Packages

| Package                 | Description                                                      |
| ----------------------- | ---------------------------------------------------------------- |
| **franka\_ros2**        | Official ROS 2 stack for controlling Franka robots.              |
| **moveit\_servo**       | Real-time Cartesian control for teleoperation.                   |
| **leap\_hand**          | URDF description and integration for the LEAP Hand end-effector. |
| **ros2\_module**        | LEAP Hand API ROS 2 wrapper.                                     |
| **dex\_retargeting**    | Retargets human hand poses to the LEAP Hand kinematics.          |
| **fr3\_leap\_teleop**   | Publishes wrist and hand pose topics for teleoperation.          |
| **fr3\_leap\_recorder** | Records multimodal data streams for later analysis or training.  |
| **easy\_handeye2**      | Visual calibration using ArUco markers and RGB-D camera.         |
| **tact9d**              | ROS 2 integration of the Daimon Robotics 9DTact tactile sensor.  |

---

## 📖 Documentation

The full **step-by-step guide** is available in [`docs/guide.md`](docs/guide.md) *(optional if you move your guide there)*.
It covers:

* Hardware setup
* Software installation
* Launching teleoperation
* Integrating visual and tactile feedback
* Recording and replaying multimodal data

---

## 🧩 Dependencies

* **ROS 2** Humble or Iron
* **libfranka** ≥ 0.12
* **MoveIt 2** + MoveIt Servo
* **RealSense SDK** (optional, for visual feedback)
* **DexRetargeting** (included and wrapped)
* **HTC Vive Ultimate Tracker SDK** (for wrist tracking)
