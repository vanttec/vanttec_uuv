#!/usr/bin/env python
# -- coding: utf-8 --

"""
Simple script that publishes a node and that 
"""

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ShapeDetectionNode:
    """ Node that takes raw image and filters it"""
    
    def __init__(self):
        self.image = None
        self.loop_rate = rospy.Rate(3)

        
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('frontr200/camera/color/image_raw', Image, 
                                    self.process_image_callback)
        
        self.pub = rospy.Publisher('processed_image_adrian', Image, queue_size=5)


    def process_image_callback(self, image_msg): 
        self.image = self.bridge.imgmsg_to_cv2(image_msg)

    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo('Publishing image')
            if self.image is not None:
                self.pub.publish(self.bridge.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('shape_detection_node', anonymous=True)
    node = ShapeDetectionNode()
    try:
        node.start()
    except rospy.exceptions.ROSInterruptException:
        pass