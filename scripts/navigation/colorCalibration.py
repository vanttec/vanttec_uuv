#!/usr/bin/env python
# -- coding: utf-8 --

"""
Module to calibrate node 
"""

import rospy
from cv_bridge import CvBridge 

from vanttec_uuv.msg import ColorRanges

import numpy as np
import cv2

class ColourCalibration:
    """ Node to calibrate range of colours """
    
    def _init_(self):
        self.rate = rospy.Rate(20)
        self.pub =  rospy.Publisher('/color_range', ColorRanges, queue_size=10)

        self.range_msg = ColorRanges()
        self.range_msg.lower = [0, 0, 0]
        self.range_msg.upper = [255, 255, 255]

    def _updater_factory(self, array, index):
        """ Creates an updater funciton that updates a value of the range 
           given the array: 'upper' or 'lower' and the index of such array 0<=i<=2
        """
        
        assert type(array) == str
        assert array.lower() == 'lower' or array.lower() == 'upper'
        
        def _updater(value):
            if array == 'lower':
                self.range_msg.lower[index] = value
            if array == 'upper':
                self.range_msg.upper[index] = value
        
        return _updater
    
    def start(self):
        
        cv2.namedWindow('Calibration node')

        # lower 
        cv2.createTrackbar('Lower B', 'Calibration node', 0, 255, self._updater_factory('lower', 0))
        cv2.createTrackbar('Lower G', 'Calibration node', 0, 255, self._updater_factory('lower', 1))
        cv2.createTrackbar('Lower R', 'Calibration node', 0, 255, self._updater_factory('lower', 2))
        # upper
        cv2.createTrackbar('Upper B', 'Calibration node', 255, 255, self._updater_factory('upper', 0))
        cv2.createTrackbar('Upper G', 'Calibration node', 255, 255, self._updater_factory('upper', 1))
        cv2.createTrackbar('Upper R', 'Calibration node', 255, 255, self._updater_factory('upper', 2))

        while not rospy.is_shutdown():
            if cv2.waitKey(1) & 0xFF == ord('q') :
                break
            
            rospy.loginfo(self.range_msg)
            self.pub.publish(self.range_msg)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('colour_calibration_node', anonymous=True)
    node = ColourCalibration()
    try: 
        node.start()
    except rospy.exceptions.ROSInterruptException:
        pass