## Dockerfile.rpi
## ROS 2 Humble teleop environment for SO-ARM on Raspberry Pi 5

FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# Base tools + ROS build / runtime dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    python3-pip \
    python3-serial \
    python3-numpy \
    python3-pil \
    python3-gi \
    gir1.2-gstreamer-1.0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    ros-humble-robot-state-publisher \
    ros-humble-v4l2-camera \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf-transformations \
    ros-humble-control-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-moveit-msgs \
 && rm -rf /var/lib/apt/lists/*

# Prepare rosdep inside the image so it's ready for workspaces
RUN rosdep init || true && rosdep update

# Python libs used by motion interface + hardware driver
RUN pip3 install --no-cache-dir \
    websockets \
    feetech-servo-sdk

WORKDIR /ws

# Default: drop into a shell; user will source ROS & workspace
CMD ["bash"]
