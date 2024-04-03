FROM osrf/ros:noetic-desktop-full

# OS: Installation of necessary packages. Always apt-get update with rm -r /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=noninteractive
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    python3-pip \
    git \
    nano \
    terminator \
    curl \
    build-essential \
    dbus-x11 \
    software-properties-common -y \
    && rm -r /var/lib/apt/lists/*

# Sourcing ROS on the bashrc for autocompletion
COPY bashrc /home/uuv/.bashrc

# ROS: installation process: http://wiki.ros.org/noetic/Installation/Ubuntu
RUN rosdep update