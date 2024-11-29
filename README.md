<h1 style="font-size: 3em;">ROS 2 Integration for Franka Robotics Research Robots</h1>

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

> **Note:** _franka_ros2_ is not officially supported on Windows.

#### Table of Contents
- [About](#about)
- [Caution](#caution)
- [Optional .bashrc Settings](#optional-bashrc-settings)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
- [Test the Setup](#test-the-setup)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

# About
The **franka_ros2** repository provides a **ROS 2** integration of **libfranka**, allowing efficient control of the Franka Robotics arm within the ROS 2 framework. This project is designed to facilitate robotic research and development by providing a robust interface for controlling the research versions of Franka Robotics robots.

We provide a Dockerfile and docker-compose.yml files for convenience. While it is possible to build **franka_ros2** directly on your local machine, doing so requires installing some dependencies manually, while hundreds of others will be managed and installed automatically by the **ROS 2** build system (e.g., via **rosdep**). This process can result in a large number of libraries being installed directly onto your system. Using Docker encapsulates these dependencies within the container, reducing the potential for conflicts. Additionally, Docker ensures a consistent and reproducible build environment across systems. For these reasons, we recommend using Docker.

# Caution
This package is in rapid development. Users should expect breaking changes and are encouraged to report any bugs via [GitHub Issues page](https://github.com/frankaemika/franka_ros2/issues).



# Setup





## Local Machine Installation
1. *Install ROS2 Development environment*

    _**franka_ros2**_ is built upon _**ROS 2 Humble**_.  

    To set up your ROS 2 environment, follow the official _**humble**_ installation instructions provided [**here**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). 
    The guide discusses two main installation options: **Desktop** and **Bare Bones**.

    #### Choose **one** of the following:
    - **ROS 2 "Desktop Install"** (`ros-humble-desktop`)  
      Includes a full ROS 2 installation with GUI tools and visualization packages (e.g., Rviz and Gazebo).  
      **Recommended** for users who need simulation or visualization capabilities.

    - **"ROS-Base Install (Bare Bones)"** (`ros-humble-ros-base`)  
      A minimal installation that includes only the core ROS 2 libraries.  
      Suitable for resource-constrained environments or headless systems.
    ---
    As discussed in the installation guide, you will need to install the **Development Tools** package:
    ```bash
    sudo apt install ros-humble-dev-tools
    ```
    Installing the **Desktop** or **Bare Bones** should automatically source the **ROS2** environment but, under some circumastances you may need to do this again:
    ```bash
    source /opt/ros/humble/setup.bash
    ```

2. *Create a ROS 2 Workspace:*
   ```bash
   mkdir -p ~/franka_ros2_ws/src
   cd ~/franka_ros2_ws  # NOT src
   ```
3. *Clone the Repositories:*
   ```bash
    source /opt/ros/humble/setup.bash
    cd ~/franka_ros2_ws
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2
    git clone https://github.com/frankaemika/franka_description.git src/franka_description
    ``` 
4. *Detect and install project dependencies*
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```
5. *Build*
   ```bash
   # use the --symlinks option to reduce disk usage, and facilitate development.
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
6. *Adjust Enviroment*
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
   ```

## Docker Container Installation
The **franka_ros2** package includes a `Dockerfile` and a `.devcontainer` directory (and also a docker-compose.yml), which allows you to use Franka ROS 2 packages without manually installing **ROS2**. 

For detailed instructions, on preparing VSCode to use the `.devcontainer` follow the setup guide from [VSCode devcontainer_setup](https://code.visualstudio.com/docs/devcontainers/tutorial).

1. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p franka_ros2_ws/src
   cd franka_ros2_ws  # Not into src
   ```
2. **Clone the Repositories:**
    ```bash
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2
    git clone https://github.com/frankaemika/franka_description.git src/franka_description
    ```
3. **Prepare to use the Container**

    Copy the .devcontainer Folder:**

    In order to use the provided Dockerfile, from within VSCode, you will probably want to copy the `.devcontainer` directory into the current directory

    This step **also** ensures that both the _*franka_ros2*_ and _*franka_description*_ folders are accessible within the DevContainer environment.
    ```bash
    cp -r src/franka_ros2/.devcontainer .
    ```
4. **Open VSCode and open the Current Folder in DevContainer**

5. **Rebuild and Reopen workspace within the Container**

    Press Ctrl + Shift + P and type: `Dev Containers: Rebuild and Reopen in Container`.

    > This command *should* result in the `rosdep` command being run during container build.  Subsequent opening of the container will not do this.

6. **Open the Terminal in VSCode:**

    Press Ctrl + (backtick). 
7. **Source the Environment:**
    ```bash
    source /opt/ros/humble/setup.sh  
    ```
8. **Build and "local install" the Franka ROS 2 Packages:**
    ```bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
    ```
8. *Adjust Enviroment*
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
   ```    


# Test the build
   ```bash
   colcon test
   ```
> Remember, franka_ros2 is under development.  
> Warnings can be expected.  
> Errors? Well, they’re just undocumented features !".

# Run a sample ROS2 application

To verify that your setup works correctly without a robot, you can run the following command to use dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```


# Troubleshooting
#### `libfranka: UDP receive: Timeout error`

If you encounter a UDP receive timeout error while communicating with the robot, avoid using Docker Desktop. It may not provide the necessary real-time capabilities required for reliable communication with the robot. Instead, using Docker Engine is sufficient for this purpose.

A real-time kernel is essential to ensure proper communication and to prevent timeout issues. For guidance on setting up a real-time kernel, please refer to the [Franka installation documentation](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).

# Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](https://github.com/frankaemika/franka_ros2/blob/humble/CONTRIBUTING.md) for more details on how to contribute to this project.

## License

All packages of franka_ros2 are licensed under the Apache 2.0 license.

## Contact 

For questions or support, please open an issue on the [GitHub Issues](https://github.com/frankaemika/franka_ros2/issues) page.

See the [Franka Control Interface (FCI) documentation](https://frankaemika.github.io/docs) for more information.


[def]: #docker-container-installation