# Use ROS 2 Humble base image
FROM ros:humble

# Set the environment to use bash
SHELL ["/bin/bash", "-c"]

# Set the working directory inside the container
WORKDIR /workspace

# Install necessary dependencies including curl
RUN apt-get update && apt-get install -y \
    curl \
    git \
    build-essential \
    xterm \
    alsa-utils \
    mesa-utils \
    python3-colcon-common-extensions \
    python3-pip \
    x11-utils \
    x11-xserver-utils \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-broadcaster \
    ros-humble-ros2-control \
    ros-humble-joint-trajectory-controller \
    ros-humble-position-controllers \
    ros-humble-effort-controllers \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 Humble repository sources for missing packages
RUN echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Create the ROS workspace and src directory
RUN mkdir -p /workspace/src


# Copy your ROS 2 project into the container
COPY ./src /workspace/src/

# Install any Python dependencies (e.g., from requirements.txt)
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Copy the start.sh script into the container
COPY scripts/start.sh /workspace/start.sh
RUN chmod +x /workspace/start.sh

# Set up entrypoint for the ROS 2 workspace
WORKDIR /workspace
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Install additional ROS 2 dependencies using rosdep (skip if no packages to install)
RUN apt-get update

RUN rosdep update

# Install additional ROS 2 dependencies using rosdep
# If you don't have ROS packages to install yet, you can comment this out
RUN rosdep install --from-paths /workspace/src --ignore-src -r -y || echo "No ROS packages to install."

ENV _NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# Set the entrypoint to the start.sh script
CMD ["/workspace/start.sh"]
