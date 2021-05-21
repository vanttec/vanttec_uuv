#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped
from vanttec_uuv.msg import GuidanceWaypoints
from usv_perception.msg import obj_detected, obj_detected_list
from nav_msgs.msg import Path

# Class Definition
class GateMission:
    def __init__(self):
        self.ned_x = 0
        self.ned_y = 0
        self.ned_z = 0
        self.yaw = 0
        self.objects_list = []
        self.activated = True
        self.state = -1
        self.distance = 0
        self.InitTime = rospy.Time.now().secs
        self.offset = .55 #camera to ins offset
        self.target_x = 0
        self.target_y = 0
        self.ned_alpha = 0
        self.choose_side = 'left'
        self.distance_away = 5
        self.waypoints = GuidanceWaypoints()
        self.uuv_path = Path()
        self.heading_threshold = 0.01
        self.depth_threshold = 0.
        
       
        #Waypoint test instead of perception node


        # ROS Subscribers
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        '''
        rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.objs_callback)
        '''

        # ROS Publishers
        self.uuv_waypoints = rospy.Publisher("/uuv_guidance/guidance_controller/waypoints", GuidanceWaypoints, queue_size=10)
        self.uuv_path_pub = rospy.Publisher("/uuv_planning/motion_planning/desired_path", Path, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)

        #Waypoint test instead of perception node

        self.objects_list = [
            {
                'X': 7,
                'Y': -4,
                'Z': 0
            }        
        ] 
    
    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z                                   
    def gatemission(self):
        self.waypoints.waypoint_list_length = 2
        self.waypoints.guidance_law = 0
        #self.waypoints.depth_setpoint = 4
        #depth_error = self.ned_z - self.waypoints.depth_setpoint
        
        if(self.state == -1):
            self.waypoints.heading_setpoint = -math.pi/4
            heading_error = self.yaw - self.waypoints.heading_setpoint
            rospy.logwarn(heading_error)
            if (heading_error < self.heading_threshold):
                self.state = 2
            self.waypoints.waypoint_list_x = [self.ned_x, 0]
            self.waypoints.waypoint_list_y = [self.ned_y, 0]
            self.waypoints.waypoint_list_z = [0,0]
            self.desired(self.waypoints)
        elif(self.state == 2):
            rospy.logwarn(self.waypoints.heading_setpoint)
            if (self.waypoints.heading_setpoint >= math.pi/4):
                self.state = 2.1
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint += math.pi/400.0
                self.waypoints.waypoint_list_x = [self.ned_x, 0]
                self.waypoints.waypoint_list_y = [self.ned_y, 0]
                self.waypoints.waypoint_list_z = [0,0]
                self.desired(self.waypoints)
        elif(self.state == 2.1):
            rospy.logwarn(self.waypoints.heading_setpoint)
            if (self.waypoints.heading_setpoint <= 0):
                self.state = 3
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint -= math.pi/400.0
                self.waypoints.waypoint_list_x = [self.ned_x, 0]
                self.waypoints.waypoint_list_y = [self.ned_y, 0]
                self.waypoints.waypoint_list_z = [0,0]
                self.desired(self.waypoints)
        elif(self.state == 3):
            self.waypoints.guidance_law = 1
            _euc_distance = pow(pow(self.ned_x-7,2)+pow(self.ned_y-0.5,2),0.5)
            if(_euc_distance <0.35):
                self.state = 6
                self.waypoints.guidance_law = 0
            else:
                self.waypoints.waypoint_list_x = [self.ned_x,7]
                self.waypoints.waypoint_list_y = [self.ned_y,0.5]
                self.waypoints.waypoint_list_z = [0,0]   
                self.desired(self.waypoints)


    def results(self):
        rospy.logwarn("Inicial ned")
        rospy.logwarn(self.ned_x)
        rospy.logwarn(self.ned_y)
        rospy.logwarn(self.ned_z)     
        rospy.logwarn(self.yaw)         


    def desired(self, path):
    	self.uuv_waypoints.publish(path)
        self.uuv_path.header.stamp = rospy.Time.now()
        self.uuv_path.header.frame_id = "world"
        del self.uuv_path.poses[:]
        for index in range(path.waypoint_list_length):
            pose = PoseStamped()
            pose.header.stamp       = rospy.Time.now()
            pose.header.frame_id    = "world"
            pose.pose.position.x    = path.waypoint_list_x[index]
            pose.pose.position.y    = path.waypoint_list_y[index]
            pose.pose.position.z    = path.waypoint_list_z[index]
            self.uuv_path.poses.append(pose)
        self.uuv_path_pub.publish(self.uuv_path)
    def activate(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.activated:
            rospy.loginfo(self.state ) 
            if(self.state != 6):
                rospy.loginfo("Gatemission is activated")
                self.gatemission()
            else:
                self.activated = False
            rate.sleep()
def main():
    rospy.init_node("gate_mission", anonymous=False)
    rate = rospy.Rate(20)
    autoNav = GateMission()
    last_detection = []
    while not rospy.is_shutdown() and autoNav.activated:
        rospy.loginfo(autoNav.state )
        if(autoNav.state != 6):
            rospy.loginfo("Gatemission is activated")
            autoNav.gatemission()
        else:
            autoNav.results()
        rate.sleep()
    rospy.spin()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
