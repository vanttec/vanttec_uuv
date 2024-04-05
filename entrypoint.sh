#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash

mkdir -p /home/uuv/vanttec_uuv/src

cd /home/uuv/vanttec_uuv

catkin_make

echo "Welcome to the Vanttec uuv's docker container. It runs with ROS Noetic. "

exec $@