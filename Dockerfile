# ──────────────────────────────────────────────────────────────────────────────
# AMR Burger — ROS 2 Jazzy + Gazebo Harmonic (Ubuntu Noble 24.04)
# Gazebo Harmonic is the default paired simulator for Jazzy on Noble.
# ──────────────────────────────────────────────────────────────────────────────
FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-lc"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo Harmonic + ROS 2 bridge (native on Noble, no extra repo needed)
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    # Robot description
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    # Navigation / msgs
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    # Visualization
    ros-jazzy-rviz2 \
    # Teleop
    ros-jazzy-teleop-twist-keyboard \
    # TF
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    # Python
    python3-colcon-common-extensions \
    python3-pip \
    python3-scipy \
    python3-numpy \
    python3-transforms3d \
    git nano wget \
    && rm -rf /var/lib/apt/lists/*

# Configure pip to allow system-wide installs if needed later
RUN pip3 config set global.break-system-packages true

# Workspace skeleton (src populated via mount at runtime)
ENV ROS_WS=/ros2_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

RUN echo "source /opt/ros/jazzy/setup.bash"                                        >> ~/.bashrc \
 && echo "[ -f $ROS_WS/install/setup.bash ] && source $ROS_WS/install/setup.bash" >> ~/.bashrc

CMD ["bash"]