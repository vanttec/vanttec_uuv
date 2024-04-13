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

# ROS: installation process: http://wiki.ros.org/noetic/Installation/Ubuntu
RUN rosdep update

# Sourcing ROS on each /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /home/uuv/vanttec_sim/devel/setup.bash" >> /root/.bashrc

RUN mkdir -p /home/uuv/vanttec_uuv/src
RUN mkdir -p /home/uuv/vanttec_sim/src