#!/usr/bin/env python
# -- coding: utf-8 --
import rospy
from std_msgs.msg import Float32MultiArray, Float32, Float64

def distance_talker():
    pub = rospy.Publisher('distance', Float64, queue_size=10)
    rospy.init_node('distance_talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    randarray = [0.01,2.4,5.3,3.3,5.78,9.78]
    i = 0
    while not rospy.is_shutdown():
        rospy.loginfo(randarray[i])
        pub.publish(randarray[i])
        i = i + 1
        if i == 5:
            i = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        distance_talker()
    except rospy.ROSInterruptException:
        pass