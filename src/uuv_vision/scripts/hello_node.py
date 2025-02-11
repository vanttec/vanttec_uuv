#!/usr/bin/env python3
#########################################################################################################
#   @Description: This is a simple script used just for testing ZED2 camera using the API.
#
#   @Author: Ivan Diaz
#   @Email: ivan.d557d@hotmail.com
#   @Date: 06.06.24
#   @Brief: It works with ZED SDK 4.1 and ROS Noetic
#
#################################################################################################
import pyzed.sl as sl
import rospy

if __name__=="__main__":
    # Create a camera object
    zed = sl.Camera()
    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720 # HD720 (1280*720pi) is the best match for the synth data resolution
    init_params.camera_fps = 30
    # Opens the camera, checks the hardware requirements and runs a self-calibration.
    error = zed.open(init_params)
    if error != sl.ERROR_CODE.SUCCESS:
        exit(1)

    rospy.init_node("hello_node")
    rospy.loginfo("Hello node just started.")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("Hellooo")
        rate.sleep()

    zed.close()
