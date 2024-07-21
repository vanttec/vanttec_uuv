#########################################################################################################
#   @Description: This Dockerfile takes the image from dustynv/zed:r32.7.1 and   
#   The most important aspect on this image is that no Nvidia GPU is required; for this reason, the 
#   zed-open-capture project is used on top of a ros noetic image with Ubuntu 20.04. For more information 
#   about dockerfiles: https://www.stereolabs.com/docs/docker/configure-ros-dockerfile
#
#   @Author: Ivan Diaz
#   @Email:  ivan.d557d@hotmail.com
#   @Date:   18.06.24
#   @Brief:  Outputs a container with Ubuntu 20.04, ROS Noetic and the zed-open-capture project.
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
