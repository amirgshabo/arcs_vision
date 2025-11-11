FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
