FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

WORKDIR /workspaces

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir -p -m 0700 /run/$USERNAME/"${USER_UID}" \
    && mkdir -p -m 0700 /run/$USERNAME/"${USER_UID}"/gdm \
    && chown $USERNAME:$USERNAME /run/$USERNAME/"${USER_UID}" \
    && chown $USERNAME:$USERNAME /workspaces \
    && chown $USERNAME:$USERNAME /run/$USERNAME/"${USER_UID}"/gdm \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libignition-gazebo6-dev \
    libpoco-dev \
    ros-humble-ackermann-msgs \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-controller-interface \
    ros-humble-controller-manager \
    ros-humble-gazebo-ros \
    ros-humble-generate-parameter-library \
    ros-humble-geometry-msgs \
    ros-humble-hardware-interface \
    ros-humble-hardware-interface-testing \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher-gui \
    ros-humble-launch-testing \
    ros-humble-moveit \
    ros-humble-realtime-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-control-test-assets \
    ros-humble-ros2-controllers \
    ros-humble-sensor-msgs \
    ros-humble-xacro \
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /etc/apt/keyrings 

RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc

RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
| sudo tee /etc/apt/sources.list.d/robotpkg.list

RUN apt-get update \
    && apt-get install -qqy robotpkg-pinocchio
    

RUN mkdir ~/source_code
RUN cd ~/source_code && git clone https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && git checkout 0.14.1 \
    && git submodule init \
    && git submodule update \
    && mkdir build && cd build \
    && cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/opt/openrobots/lib -DCMAKE_PREFIX_PATH=/opt/openrobots/ .. \
    && make franka -j$(nproc) \
    && cpack -G DEB \
    && sudo dpkg -i libfranka*.deb

RUN addgroup realtime
RUN usermod -a -G realtime $USERNAME

ENV SHELL "/bin/bash"
ENV XDG_RUNTIME_DIR=/run/user/"${USER_UID}"

# set the default user to the newly created user
USER $USERNAME
CMD ["/bin/bash"]

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/workspaces/src/" >> ~/.bashrc