#########################################################################################################
#   @Description: This Dockerfile takes the image from dustynv/ros:noetic-pytorch-l4t-r32.7.1. In this case,
#   the ZED SDK is installed manually outside the Dockerfile. The matching version according to the Jetson 
#   TX2 is the one for L4T 32.7.1 which corresponds to the Jetson Pack 4.6.1.
#
#   @Author: Ivan Diaz
#   @Email:  ivan.d557d@hotmail.com
#   @Date:   18.06.24
#   @Brief:  Outputs a container with Ubuntu 20.04 and ROS Noetic
#
#########################################################################################################
FROM dustynv/ros:noetic-pytorch-l4t-r32.7.1

# Install useful packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y \
    python3-pip \
    git \
    nano \
    terminator \
    curl \
    wget \
    build-essential \
    dbus-x11 \
    zstd \
    software-properties-common -y \
    && rm -r /var/lib/apt/lists/*

# Sourcing ROS on each /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /ws/vanttec_uuv/devel/setup.bash" >> /root/.bashrc
RUN echo "$(printf 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/ws/vanttec/build\n')" >> /root/.bashrc
RUN echo "export OPENBLAS_CORETYPE=ARMV8" >> /root/.bashrc

RUN mkdir -p /ws/vanttec_uuv/src
