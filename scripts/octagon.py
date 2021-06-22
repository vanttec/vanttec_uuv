#!/usr/bin/env python
# -- coding: utf-8 --

import math
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped
from vanttec_uuv.msg import GuidanceWaypoints
from nav_msgs.msg import Path

# Class Definition
class OctagonMission:
    def _init_(self):
        self.ned_x = 0
        self.ned_y = 0
        self.ned_z = 0
        self.yaw = 0
        self.objects_list = []
        self.activated = True
        self.state = -1
        self.searchstate = -1
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
        #findimage works as a counter however with vision input once it identifies the target it should set this variable to True
        self.findimage = 0
        self.searchx = 0.0
        self.searchy = 0.0
        self.searchz = 0.0
        self.sweepstate = -1
        self.foundstate = -1
        self.enterwaypoint = 0.0
        self.leavewaypoint = 0.0
        self.ylabel = 0.0
        self.prevstate ="RIGHT"
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
        #This array shall be modified with zed inputs of distance    
    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z      
    def sweep(self,nextmission):
        self.waypoints.guidance_law = 0
        #self.waypoints.depth_setpoint = 4
        #depth_error = self.ned_z - self.waypoints.depth_setpoint
        if(self.sweepstate == -1):
            if (self.waypoints.heading_setpoint <=  -math.pi/4):
                self.sweepstate = 2
            self.waypoints.heading_setpoint -= math.pi/400.0
            self.waypoints.waypoint_list_x = [0, 0]
            self.waypoints.waypoint_list_y = [0, 0]
            self.waypoints.waypoint_list_z = [0,0]
            self.desired(self.waypoints)
        elif(self.sweepstate == 2): 
            if (self.waypoints.heading_setpoint >= math.pi/4):
                self.sweepstate = 2.1
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint += math.pi/400.0
                self.waypoints.waypoint_list_x = [0 ,0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0,0]
                self.desired(self.waypoints)
        elif(self.sweepstate == 2.1): 
            if (self.waypoints.heading_setpoint <= 0):
                self.waypoints.guidance_law = 0
                self.searchstate = nextmission
                self.searchx = self.ned_x + 2
                self.searchy = self.ned_y
                self.sweepstate = -1
                self.desired(self.waypoints)
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint -= math.pi/400.0
                self.waypoints.waypoint_list_x = [0, 0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0,0]
                self.desired(self.waypoints) 
           
    def grab_bottle(self,nextmission):
        self.waypoints.guidance_law = 0
        #self.waypoints.depth_setpoint = 4
        #depth_error = self.ned_z - self.waypoints.depth_setpoint
        if(self.sweepstate == -1):
            if (self.waypoints.heading_setpoint <=  -math.pi/8):
                self.sweepstate = 2
            self.waypoints.heading_setpoint -= math.pi/400.0
            self.waypoints.waypoint_list_x = [0, 0]
            self.waypoints.waypoint_list_y = [0, 0]
            self.waypoints.waypoint_list_z = [0,0]
            self.desired(self.waypoints)
        elif(self.sweepstate == 2): 
            if (self.waypoints.heading_setpoint >= math.pi/8):
                self.sweepstate = 2.1
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint += math.pi/400.0
                self.waypoints.waypoint_list_x = [0 ,0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0,0]
                self.desired(self.waypoints)
        elif(self.sweepstate == 2.1): 
            if (self.waypoints.heading_setpoint <= 0):
                self.waypoints.guidance_law = 0
                self.foundstate = nextmission
                self.searchx = self.ned_x + 3
                self.searchy = self.ned_y
                self.sweepstate = -1
                self.desired(self.waypoints)
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint -= math.pi/400.0
                self.waypoints.waypoint_list_x = [0, 0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0,0]
                self.desired(self.waypoints)       
    def search(self):
        #look subscriber of image distance
        if(self.findimage <= 1):
            rospy.logwarn("Searching image")
            if self.searchstate == -1:
                #sweep to find 
                rospy.logwarn("still searching")
                self.sweep(0)
            elif self.searchstate ==0:
                self.waypoints.guidance_law = 1
                #move 2 meter
                _euc_distance = pow(pow(self.ned_x-self.searchx,2)+pow(self.ned_y-self.searchy,2),0.5)
                if(_euc_distance <0.35):
                    self.findimage += 1  
                    self.searchstate = -1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.searchx]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.searchy]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
        else:
            rospy.logwarn("Found image")
            rospy.logwarn(self.foundstate)
            if(self.foundstate == -1):
                self.samex = self.ned_x
                self.righty = self.ned_y+2.5
                self.lefty = self.ned_y-2.5
                self.downz = self.ned_z+1
                self.upz = self.ned_z
                self.leavewaypointxcenter = self.ned_x+2
                self.leavewaypointycenter = self.ned_y 
                self.leavewaypointx = self.ned_x+2
                self.leavewaypointy = self.ned_y 
                self.foundstate = 0
            if(self.foundstate == 0):
               #stay find the best position to grab bottle
               rospy.logwarn("Looking the best position to grab bottle") 
               self.grab_bottle(1.1)
            #Grab bottle from dollar table
            elif(self.foundstate == 1.1):
               #descend to grab bottle
                self.waypoints.guidance_law = 1
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_z-self.downz,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 1.2
                    self.waypoints.guidance_law = 0
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,0]
                    self.waypoints.waypoint_list_z = [self.ned_z,self.downz]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 1.2):
                self.waypoints.guidance_law = 1
               #surge with bottle
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_z-self.upz,2),0.5)
                if(_euc_distance <0.35):
                    #It will move to left or right depending on initial decisition
                    if(self.prevstate =="LEFT"):
                        self.foundstate = 2
                    else:
                        self.foundstate = 2.1
                    self.waypoints.guidance_law = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,0]
                    self.waypoints.waypoint_list_z = [self.ned_z,self.upz] 
                    self.desired(self.waypoints)   
            elif(self.foundstate == 2):
               #move to dollar sign LEFT
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_y-self.lefty,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 3
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.lefty]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 2.1):
               #move to axe sign RIGHT
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_y-self.righty,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 3
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.righty]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 3):
               rospy.logwarn("Looking the best position to grab bottle") 
               self.grab_bottle(3.1) 
            elif(self.foundstate == 3.1):
                self.waypoints.guidance_law = 1
               #descend to put bottle
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_z-self.downz,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 3.2
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,0]
                    self.waypoints.waypoint_list_z = [self.ned_z,self.downz]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 3.2):
                self.waypoints.guidance_law = 1
                #surge without bottle
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_z-self.upz,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 3.3
                    self.waypoints.guidance_law = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,0]
                    self.waypoints.waypoint_list_z = [self.ned_z,self.upz] 
                    self.desired(self.waypoints)                   
            elif(self.foundstate == 3.3):
               #move to leave waypoint
                _euc_distance = pow(pow(self.ned_x-self.leavewaypointxcenter,2)+pow(self.ned_y-self.leavewaypointycenter,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 4
                    self.waypoints.guidance_law = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leavewaypointxcenter]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leavewaypointycenter]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)     
            elif(self.foundstate == 4):
               #move to leave waypoint
                _euc_distance = pow(pow(self.ned_x-self.leavewaypointx,2)+pow(self.ned_y-self.leavewaypointy,2),0.5)
                if(_euc_distance <0.35):
                    self.state = 6
                    self.waypoints.guidance_law = 0
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leavewaypointx]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leavewaypointy]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)  

    def octagonmission(self):
        self.waypoints.waypoint_list_length = 2
        self.waypoints.guidance_law = 1
        #self.waypoints.depth_setpoint = 4
        #depth_error = self.ned_z - self.waypoints.depth_setpoint
        
        if(self.state == -1):
            self.search()

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
    def results(self):
        rospy.logwarn("Inicial ned")
        rospy.logwarn(self.ned_x)
        rospy.logwarn(self.ned_y)
        rospy.logwarn(self.ned_z)     
        rospy.logwarn(self.yaw)         



    def activate(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.activated:
            rospy.loginfo(self.state ) 
            if(self.state != 6):
                rospy.loginfo("Octagonmission is activated")
                self.octagonmission()
            else:
                self.activated = False
            rate.sleep()
def main():
    rospy.init_node("octagon_mission", anonymous=False)
    rate = rospy.Rate(20)
    octagon_mission = OctagonMission()
    last_detection = []
    while not rospy.is_shutdown() and octagon_mission.activated:
        if(octagon_mission.state != 6):
            rospy.loginfo("Octagonmission is activated")
            octagon_mission.octagonmission()
        else:
            octagon_mission.results()
        rate.sleep()
    rospy.spin()



if __name__ == "_main_":
    try:
        main()
    except rospy.ROSInterruptException:
        pass