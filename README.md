# ROS 2 Integration for Franka Robotics Research Robots

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

**Note:** franka_ros2 is not officially supported on Windows.



## Table of Contents
- [About](#about)
- [Caution](#caution)
- [Prerequisites](#prerequisites)
- [Optional .bashrc Settings](#optional-bashrc-settings)
- [Setup](#setup)
  - [Install From Source](#install-from-source)
  - [Use VSCode DevContainer](#use-vscode-devcontainer)
- [Troubleshooting](#troubleshooting)
  - [Error: Docker Unable to Access /tmp/.X11-unix](#error-docker-unable-to-access-tmpx11-unix)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About
The **franka_ros2** repository provides a **ROS 2** integration of **libfranka**, allowing efficient control of the Franka Robotics arm within the ROS 2 framework. This project is designed to facilitate robotic research and development by providing a robust interface for controlling the research versions of Franka Robotics robots.

## Caution
This package is in rapid development. Users should expect breaking changes and are encouraged to report any bugs via [GitHub Issues page](https://github.com/frankaemika/franka_ros2/issues).

## Prerequisites
Before installing **franka_ros2**, ensure you have the following prerequisites:
- **ROS 2 Humble Installation:** You can install [`ros-humble-desktop`](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)  or use VSCode IDE with DevContainer. 
- **PREEMPT_RT Kernel (optional but recommended):** A real-time kernel is necessary for the cartesian_pose, joint_position, and elbow_position command interfaces.
- **System-wide libfranka Installation:** 
    - If you plan to **install from source**, a libfranka installation is required. Please refer to the [libfranka repository](https://github.com/frankaemika/libfranka) for detailed build steps.
    - If you are **using the DevContainer**, you do not need to install libfranka system-wide, as it will be included in the container.

    Regardless of your setup, it is important to check the compatibility of your Robot OS version with libfranka to avoid potential errors. For detailed compatibility information, please consult the [libfranka compatibility matrix](https://frankaemika.github.io/docs/compatibility.htmlk-to-matrix).

## Optional .bashrc Settings
Enhance your development experience by adding the following lines to your `.bashrc` file:

```bash
# Enable colorized warn and error messages
export RCUTILS_COLORIZED_OUTPUT=1

# Set system language to American English to avoid issues in RViz
export LC_NUMERIC=en_US.UTF-8  
```

## Setup

### Install From Source

1. **Install Required Packages:**
   ```bash
   sudo apt install -y \
   ros-humble-ament-cmake \
   ros-humble-ament-cmake-clang-format \
   ros-humble-angles \
   ros-humble-ros2-controllers \
   ros-humble-ros2-control \
   ros-humble-ros2-control-test-assets \
   ros-humble-controller-manager \
   ros-humble-control-msgs \
   ros-humble-control-toolbox \
   ros-humble-generate-parameter-library \
   ros-humble-joint-state-publisher \
   ros-humble-joint-state-publisher-gui \
   ros-humble-moveit \
   ros-humble-pinocchio \
   ros-humble-realtime-tools \
   ros-humble-xacro \
   ros-humble-hardware-interface \
    ros-humble-ros-gz \
   python3-colcon-common-extensions
   ```


2. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p ~/franka_ros2_ws/src
   ```
3. **Clone the Repositories and Build Packages:**
   ```bash
    source /opt/ros/humble/setup.bash
    cd ~/franka_ros2_ws 
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2 
    git clone https://github.com/frankaemika/franka_description.git src/franka_description 
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
    source install/setup.sh
    ``` 

### Use VSCode DevContainer


The `franka_ros2` package includes a `.devcontainer` folder, which allows you to use Franka ROS 2 packages without manually installing ROS 2 or `libfranka`. For detailed instructions, follow the setup guide from [VSCode devcontainer_setup](https://code.visualstudio.com/docs/devcontainers/tutorial).

1. **Create a ROS 2 Workspace:**
   ```bash
   mkdir franka_ros2_ws
   cd franka_ros2_ws
   ```

2. **Clone the Repositories:**
    ```bash
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2 
    git clone https://github.com/frankaemika/franka_description.git src/franka_description
    ```

3. **Copy the .devcontainer Folder:**
    ```bash
    cp -r src/franka_ros2/.devcontainer .
    ```
    This step ensures that both the `franka_ros2` and `franka_description` folders are accessible within the DevContainer environment.

4. **Open VSCode:**
    ```bash
    code . 
    ```
5. **Open the Current Folder in DevContainer:**
    
    Press Ctrl + Shift + P and type: `Dev Containers: Rebuild and Reopen in Container`.


6. **Open the Terminal in VSCode:**
    
    Press Ctrl + (backtick). 

7. **Source the Environment:**
    ```bash
    source /opt/ros/humble/setup.sh  
    ```
8. **Install the Franka ROS 2 Packages:**
    ```bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
    source install/setup.sh 
    ```
## Troubleshooting
### Error: Docker Unable to Access /tmp/.X11-unix

If you encounter the following error after running **Rebuild and Reopen in Container**: 

Error response from daemon: Mounts denied:  
The path /tmp/.X11-unix is not shared from the host and is not known to Docker. 
 

This occurs because Docker cannot access the `/tmp/.X11-unix` directory on the host system. Follow these steps to fix it: 

1. **Allow Docker to Access /tmp/.X11-unix:**

    If you are using Docker Desktop, navigate to:

    1. `Docker Desktop` -> `Settings` -> `Resources` -> `File Sharing`
    2. Add `/tmp/.X11-unix` to the list of shared directories.
    3. Apply the changes and restart Docker.
 
2. **Verify Sharing Path on Linux:**

    Ensure that Docker has the correct permissions for /tmp/.X11-unix:  
    ```bash
    sudo chmod +x /tmp/.X11-unix 
    ```
 

3. **Restart Docker and re-run the container.**


## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md]() for more details on how to contribute to this project. 


## License 

All packages of franka_ros2 are licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html). 
 

## Contact 

For questions or support, please open an issue on the [GitHub Issues](https://github.com/frankaemika/franka_ros2/issues) page. 

See the [Franka Control Interface (FCI) documentation](https://frankaemika.github.io/docs) for more information.
