#!/usr/bin/env python

from std_msgs.msg import String
from imutils.video import VideoStream
from imutils.video import FPS

#from srv import DistanceCal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import imutils
import argparse
import numpy as np
import time
import rospy
import cv2
import math

import os

class Color():
    BLUE  = '\033[94m'
    GREEN = '\033[92m'
    RED  = '\033[91m'
    DONE  = '\033[0m'


class Detection_Node:
    def __init__(self):

        self.bridge = CvBridge()
        self.image = np.zeros((560,1000,3),np.uint8)
        self.depth_image = np.zeros((560,1000,3),np.uint8)
        self.depth = np.zeros((560,1000,3),np.uint8)
        self.points_list = [[0,0,0]]


        #rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.callback_zed_img)
        #rospy.Subscriber("/zed/zed_node/point_cloud/cloud_registered", PointCloud2, self.callback_zed_cp)
        rospy.Subscriber("/frontr200/camera/color/image_raw", Image, self.callback_zed_img)
        rospy.Subscriber("/frontr200/camera/depth/image_raw", Image, self.callback_zed_depth_img)

        self.invertimage=rospy.Publisher("/invert_image",Image,queue_size=10)
        self.invert_depth_image=rospy.Publisher("/invert_depth_image",Image,queue_size=10)
        # rospy.Subscriber("/frontr200/camera/depth_registered/points", PointCloud2, self.callback_zed_cp)

        #self.detector_pub = rospy.Publisher('/uuv_perception/yolo_zed/objects_detected', obj_detected_list, queue_size=10)


    def callback_zed_img(self,img):
        """ ZED rect_image callback"""
        self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        self.image = cv2.flip(self.image,-1)
        new_Image=Image()
        new_Image = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
        self.invertimage.publish(new_Image)

        
    def callback_zed_depth_img(self,img):
        """ ZED rect_image callback"""
        self.depth_image = self.bridge.imgmsg_to_cv2(img, "16UC1")
        self.depth_image = cv2.flip(self.depth_image,-1)
        new_depth_Image=Image()
        new_depth_Image = self.bridge.cv2_to_imgmsg(self.depth_image, "16UC1")
        self.invert_depth_image.publish(new_depth_Image)

if __name__ == '__main__':
    try:
        rospy.init_node('yolo_zed')
        rate = rospy.Rate(20) # 20Hz
        D = Detection_Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
