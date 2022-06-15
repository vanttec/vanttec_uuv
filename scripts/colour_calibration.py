#!/usr/bin/env python
# -- coding: utf-8 --

"""
This is a node dedicated to calibrate the colours used in the shape detection with an UI
"""

import rospy 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

def updater_factory(array, index, image, 
                    lower = np.array([0, 0, 0], dtype='uint8'), 
                    upper = np.array([0, 0, 0], dtype='uint8')):
    
    def _updater(value):
        if array == 'upper':
            upper[index] = value
        elif array == 'lower':
            lower[index] = value
        

        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask=mask)
        imgray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        
        # get centroids of images 
        
        contours, heriarchy = cv2.findContours(imgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        heriarchy = heriarchy[0]
        cv2.drawContours(output, contours, -1, (0,255,0), 3)

        for cont in contours:
            moments = cv2.moments(cont)
            n = moments['m00'] if moments['m00'] else 1
            cx = int(moments['m10']/n)
            cy = int(moments['m01']/n)
            scv2.circle(output, (cx, cy), 10, (0, 0, 255), 1)
        
        print('HERIARCHY: ', heriarchy.shape)
        print('CONTOURS: ', print(len(contours)))
        print('CENTROID AT: ', (cx, cy))
        final = np.hstack([image, output])
        cv2.imshow('PROCESSED', final)
        
    return _updaterdef updater_factory(array, index, image, 
                    lower = np.array([0, 0, 0], dtype='uint8'), 
                    upper = np.array([0, 0, 0], dtype='uint8')):
    
    def _updater(value):
        if array == 'upper':
            upper[index] = value
        elif array == 'lower':
            lower[index] = value
        

        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask=mask)
        imgray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        
        # get centroids of images 
        
        contours, heriarchy = cv2.findContours(imgray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        heriarchy = heriarchy[0]
        cv2.drawContours(output, contours, -1, (0,255,0), 3)

        for cont in contours:
            moments = cv2.moments(cont)
            n = moments['m00'] if moments['m00'] else 1
            cx = int(moments['m10']/n)
            cy = int(moments['m01']/n)
            scv2.circle(output, (cx, cy), 10, (0, 0, 255), 1)
        
        print('HERIARCHY: ', heriarchy.shape)
        print('CONTOURS: ', print(len(contours)))
        print('CENTROID AT: ', (cx, cy))
        final = np.hstack([image, output])
        cv2.imshow('PROCESSED', final)
        
    return _updater

class ColourCalibrationNode:

    def __init__(self):
        self.image = None
        self.loop_rate = rospy.Rate(5)
        
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('frontr200/camera/color/image_raw', Image, 
                                    self.process_image)
        self.pub = rospy.Publisher('processed_image_calibration', Image, queue_size=5)
    
    def process_image(self, image_msg):
        self.image = self.bridge.imgmsg_to_cv2(image_msg)

    def start(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                self.pub.publish(self.bridge.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('colour_calibration_node', anonymous=True)
    node = ColourCalibrationNode()
    try: 
        node.start()
    except rospy.exceptions.ROSInterruptException:
        pass