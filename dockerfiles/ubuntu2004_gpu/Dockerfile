#########################################################################################################
#   @Description: This Dockerfile takes the image from stereolabs: 3.7-gl-devel-cuda11.4-ubuntu20.04. 
#   Given that cuda 11.4 was selected, the drivers should match it, i.e., ubuntu drivers 470. 
#   For more info: https://www.stereolabs.com/docs/docker/configure-ros-dockerfile
#
#   @Author: Ivan Diaz
#   @Email:  ivan.d557d@hotmail.com
#   @Date:   26.05.24
#   @Brief:  Outputs a container with Ubuntu 20.04, ZED SDK 4.1 and ROS Noetic.
#
#########################################################################################################

FROM stereolabs/zed:4.1-gl-devel-cuda11.4-ubuntu20.04

# Install and setup of ROS
RUN apt-get update -y && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata curl && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt install curl && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt-get update &&\
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop-full build-essential cmake usbutils libusb-1.0-0-dev git -y --allow-unauthenticated \
    && rm -rf /var/lib/apt/lists/*

# Install useful packages
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

# Sourcing ROS on each /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /ws/vanttec_uuv/devel/setup.bash" >> /root/.bashrc
RUN echo "$(printf 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/ws/vanttec/build\n')" >> /root/.bashrc

RUN mkdir -p /ws/vanttec_uuv/src