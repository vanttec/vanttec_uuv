#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped, Point
from vanttec_uuv.msg import GuidanceWaypoints
from nav_msgs.msg import Path

ned_x = 0
ned_y = 0
ned_z = 0
yaw = 0

def ins_pose_callback(pose):
    global ned_x,ned_y,ned_z,yaw
    ned_x = pose.position.x
    ned_y = pose.position.y
    ned_z = pose.position.z
    yaw = pose.orientation.z 

def main():
    rospy.init_node("ASMC_Guidance_Test", anonymous=False)
    rate = rospy.Rate(20)

    # ROS Subscribers
    rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, ins_pose_callback)

    # ROS Publishers
    uuv_waypoints = rospy.Publisher("/uuv_guidance/guidance_controller/waypoints", GuidanceWaypoints, queue_size=10)

    waypoints = GuidanceWaypoints()
    distance = 5

    while not rospy.is_shutdown() and (distance > 0.1):
        distance = math.sqrt(math.pow(ned_x - 5,2) + math.pow(ned_y - 5,2))
        waypoints.guidance_law = 1 # ASMC Guidance
        waypoints.waypoint_list_x = [5]
        waypoints.waypoint_list_y = [5]
        waypoints.waypoint_list_z = [0]
        waypoints.heading_setpoint = 0
        waypoints.waypoint_list_length = 1
        # rospy.loginfo("Publishing")
        uuv_waypoints.publish(waypoints)

        rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
