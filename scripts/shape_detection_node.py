#!/usr/bin/env python
# -- coding: utf-8 --

"""
Simple script that publishes a node and that 
"""

import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vanttec_uuv.msg import TargetsCentroids

import cv2 
import numpy as np

class ShapeDetectionNode:
    """ Node that takes raw image and filters it"""
    
    def __init__(self):
        # upper and lower bounds for colour of contour for image shape detection
        self.lower = np.array([0,  0,  216], dtype='uint8')
        self.upper = np.array([21, 33, 255], dtype='uint8')

        self.image = None
        self.message = TargetsCentroids()   
        
        self.message.circle = [-1, -1]
        self.message.other_shape = [-1, -1]
        
        self.loop_rate = rospy.Rate(3)
        
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('frontr200/camera/color/image_raw', Image, 
                                    self.process_image_callback)
        
        self.pub = rospy.Publisher('/torpedos/target_centroids', TargetsCentroids, queue_size=5)


    def process_image_callback(self, image_msg): 
        """ Detects the centroid of the circle shape and of the other special shape 
        and assigns it to the ros message
        """
        self.image = self.bridge.imgmsg_to_cv2(image_msg)

        mask = cv2.inRange(image, self.lower, self.upper)

        # temporral for filling 
        provitional = mask.copy()
        r, c = mask.shape()
        second_mask = np.zeros([r+2, c+2], dtype='uint8')
        cv2.floodFill(provitional, second_mask, (0, 0), 255)

        # bitwise not of provitional and then fill with or
        provitional = cv2.bitwise_not(provitional)
        mask = cv2.bitwise_or(mask)

        # identify contours
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours[0]:

            # centroids with moments of the image
            moments = cv2.moments(contour)
            c_x = int(moments['m10'] / moments['m00'])
            c_y = int(moments['m01'] / moments['m00'])

            # number of edges
            approximation = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)
            edges = len(approximation)
            if edges == 4:
                # square detected
                self.message.other_shape = [c_x, c_y]
            if edges == 10:
                # star detected
                self.message.other_shape = [c_x, c_y]
            if edgeds > 10:
                # circle detected
                self.message.circle = [c_x, c_y]
    
    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo('Publishing image')
            if self.messagee is not None:
                self.pub.publish(self.message)
            self.loop_rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('shape_detection_node', anonymous=True)
    node = ShapeDetectionNode()
    try:
        node.start()
    except rospy.exceptions.ROSInterruptException:
        pass