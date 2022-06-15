#!/usr/bin/env python
# -- coding: utf-8 --
from time import time
import rospy
from visualization_msgs.msg import Marker 

def distance_talker():
    rospy.init_node('Marker_pub', anonymous=True)
    marker = Marker()
    marker.header.frame_id = "world_ned"
    marker.id = 0
    marker.header.stamp = rospy.get_rostime()

    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1 # Don't forget to set the alpha!
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
 



    pub = rospy.Publisher('marker', Marker, queue_size=10)
    rospy.init_node('Marker_pub', anonymous=True)

    rate = rospy.Rate(100) # 10hz
    i = 0
    while not rospy.is_shutdown():
        pub.publish(marker)


if __name__ == '__main__':
    try:
        distance_talker()
    except rospy.ROSInterruptException:
        pass