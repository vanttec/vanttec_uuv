#!/usr/bin/env python
# -- coding: utf-8 --

"""
Simple script that publishes a node and that 
"""

import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vanttec_uuv.msg import TargetsCentroids, ColorRanges

import cv2
import numpy as np

max_ = max; # capture the pointer to the native python function 

class ShapeDetectionNode:
    """ Node that takes raw image and filters it"""
    
    def _init_(self):
        # upper and lower bounds for colour of contour for image shape detection
        self.lower = np.array([0,  0,  0], dtype='uint8')
        self.upper = np.array([240, 240, 255], dtype='uint8')
        
        self.image = None
        self.message = TargetsCentroids()   
        
        self.message.other_shape = [-1, -1]
        self.message.circle = [-1, -1]
        
        self.loop_rate = rospy.Rate(3)
        
        self.bridge = CvBridge()
        
        self.calibration_sub = rospy.Subscriber('/color_range', ColorRanges, 
                                                self.update_ranges)
        self.sub = rospy.Subscriber('/invert_image', Image, 
                                    self.process_image_callback)
        
        self.pub = rospy.Publisher('/torpedos/target_centroids', TargetsCentroids, queue_size=5)
        
        self.maskPub = rospy.Publisher('/shape_image', Image, queue_size=5)
        self.imagePub = rospy.Publisher('/marked_image', Image, queue_size=5)
    
    def update_ranges(self, range):
        """ Update the colour ranges for mask """
        self.lower = np.array(range.lower, dtype='uint8')
        self.upper = np.array(range.upper, dtype='uint8')
    
    def process_image_callback(self, image_msg): 
        """ Detects the centroid of the circle shape and of the other special shape 
        and assigns it to the ros message
        """

        self.image = self.bridge.imgmsg_to_cv2(image_msg)
        self.image = np.array(self.image)
        mask = cv2.inRange(self.image, self.lower, self.upper)
        
        # temporral for filling 
        provitional = mask.copy()
        r, c = mask.shape
        second_mask = np.zeros([r+2, c+2], dtype='uint8')
        cv2.floodFill(provitional, second_mask, (0, 0), 255)

        # bitwise not of provitional and then fill with or
        provitional = cv2.bitwise_not(provitional)
        mask = cv2.bitwise_or(mask, provitional)

        # identify contours
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours if len(contours) == 2 else contours[1]
        
        c_x = 0
        c_y = 0

        for contour in contours:
            # centroids with moments of the image
            moments = cv2.moments(contour)
            c_x = -1
            c_y = -1
            if moments['m00'] != 0:
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
            if edges > 10:
                # circle detected
                self.message.circle = [c_x, c_y]
        
        
        marked_image = self.image.copy()
        cv2.circle(marked_image, tuple(self.message.circle), 5, (0, 255, 0), 5)
        cv2.circle(marked_image, tuple(self.message.other_shape), 5, (0, 255, 0), 5)
        marked_image_msg = self.bridge.cv2_to_imgmsg(marked_image)
        self.imagePub.publish(marked_image_msg)

        maskmsg = self.bridge.cv2_to_imgmsg(mask)
        self.maskPub.publish(maskmsg)

    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo('Publishing image')
            if self.message is not None:
                self.pub.publish(self.message)
                rospy.loginfo(self.message)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('shape_detection_node', anonymous=True)
    node = ShapeDetectionNode()
    try:
        node.start()
    except rospy.exceptions.ROSInterruptException:
        pass