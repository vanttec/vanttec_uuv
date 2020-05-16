#!/bin/bash

cd ~/Desktop
mkdir vanttec_uuv_ws
mv vanttec_uuv vanttec_uuv_ws/src
cd vanttec_uuv_ws/src
git submodule update --init --recursive
cd ~/Desktop/vanttec_uuv_ws
catkin_make
