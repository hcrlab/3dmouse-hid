ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-${ROS_DISTRO}-ur-description \
    ros-${ROS_DISTRO}-ur-robot-driver \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ur-moveit-config \
    ros-${ROS_DISTRO}-ur-simulation-gz \
    ros-${ROS_DISTRO}-ros-gz-image \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-compressed-image-transport \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-position-controllers \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
