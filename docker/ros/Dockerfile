FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-humble-ur-description \
    ros-humble-controller-manager \
    ros-humble-ur-moveit-config \
    ros-humble-ur-simulation-gz \
    ros-humble-ros-gz-image \
    ros-humble-ros-gz-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-rosbridge-suite \
    ros-humble-ros2-control \
    ros-humble-position-controllers \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
