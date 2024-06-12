FROM osrf/ros:noetic-desktop-full 

ENV DEBIAN_FRONTEND=noninteractive

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
    cmake \
    libusb-1.0-0-dev \ 
    libhidapi-libusb0 \ 
    libhidapi-dev \
    libopencv-dev \
    libopencv-viz-dev \
    dbus-x11 \
    software-properties-common -y \
    && rm -r /var/lib/apt/lists/*

RUN git clone https://github.com/stereolabs/zed-open-capture.git

# Sourcing ROS on each /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /ws/vanttec_uuv/devel/setup.bash" >> /root/.bashrc

RUN mkdir -p /ws/vanttec_uuv/src
