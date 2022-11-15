#!/usr/bin/env python
# -- coding: utf-8 --

import math
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped, Point
from vanttec_uuv.msg import GuidanceWaypoints
from nav_msgs.msg import Path

# Class Definition
class BinMission:
    def __init__(self):
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
        self.locatemarker = False

        #Waypoint test instead of perception node


        # ROS Subscribers
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        rospy.Subscriber("/markerwaypoint",Point,self.marker_callback)
        '''
        rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.objs_callback)
        '''

        # ROS Publishers
        self.uuv_waypoints = rospy.Publisher("/uuv_guidance/guidance_controller/waypoints", GuidanceWaypoints, queue_size=10)
        self.uuv_path_pub = rospy.Publisher("/uuv_planning/motion_planning/desired_path", Path, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)

        #Waypoint test instead of perception node
        #This variable shall be modified with zed inputs of distance 
        # find image(bool)   
    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z  
    def marker_callback(self, msg):
        if(msg.x != 1 and msg.y != 0):
            self.callbackmarkerx = msg.x
            self.callbackmarkery =  msg.y
            self.callbackmarkerz = msg.z
            self.locatemarker = True    
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
                #self.searchx = self.ned_x + 2.5
                #self.searchy = self.ned_y + 1.3
                self.sweepstate = -1
                self.waypoints.guidance_law = 0
                self.searchstate = nextmission
                if(self.locatemarker == False):
                    self.searchx = self.ned_x + 2.5
                    self.searchy = self.ned_y + 0
                else:
                    self.searchx = self.markerx + self.differencemarkerx
                    self.searchy = self.markery + self.differencemarkery
                    self.findimage += 1  
                self.desired(self.waypoints)
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint -= math.pi/400.0
                self.waypoints.waypoint_list_x = [0, 0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0,0]
                self.desired(self.waypoints) 
    def grab_bin(self,nextmission):
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
                self.searchx = self.ned_x + 3.5
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
        #look subscriber of pathmarker
        if(self.locatemarker == False):
            rospy.logwarn("Pahtmarker is not located")
            if self.searchstate == -1:
                #sweep to find 
                self.sweep(0)
            elif self.searchstate == 0:
                self.waypoints.guidance_law = 1
                #move 3 meter
                _euc_distance = pow(pow(self.ned_x-self.searchx,2)+pow(self.ned_y-self.searchy,2),0.5)
                if(_euc_distance <0.35):
                    self.searchstate = -1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.searchx]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.searchy]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints) 
        #look subscriber of image distance  
        elif(self.findimage <= 0):
            rospy.logwarn("Pahtmarker is located")
            self.timewait = rospy.get_time()
            if self.searchstate == -1:
                self.markerx = self.callbackmarkerx
                self.markery = self.callbackmarkery
                self.markerz = self.callbackmarkerz
                self.differencemarkerx = self.callbackmarkerx - self.ned_x
                self.differencemarkery = self.callbackmarkery - self.ned_y
                self.differencemarkerz = self.callbackmarkerz - self.ned_z
                self.searchstate = 0
            if self.searchstate == 0:
                self.waypoints.guidance_law = 1
                #move 3 meter
                _euc_distance = pow(pow(self.ned_x-self.markerx,2)+pow(self.ned_y-self.markery,2),0.5)
                if(_euc_distance <0.35):
                    self.searchstate = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.markerx]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.markery]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif self.searchstate ==1: 
                #sweep to find 
                self.sweep(0)
        else:
            rospy.logwarn("Found image")
            rospy.logwarn(self.foundstate)
            if(self.foundstate == -1):
                self.samex = self.ned_x
                self.righty = self.ned_y+1.0
                self.lefty = self.ned_y-1.0
                self.downz = self.ned_z+1.5
                self.upz = self.ned_z
                self.leavewaypointxcenter = self.ned_x+2
                self.leavewaypointycenter = self.ned_y 
                self.leavewaypointx = self.ned_x+2
                self.leavewaypointy = self.ned_y 
                self.foundstate = 0
            elif(self.foundstate == 0):
                self.waypoints.guidance_law = 1
               #move to right bin
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_y-self.righty,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 0.1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.righty]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 0.1):
               #stay find the best position to grab bin
               rospy.logwarn("Looking the best position to grab bin") 
               self.grab_bin(1.1)
            elif(self.foundstate == 1.1):
               #descend
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
               #ascend
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_z-self.upz,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 2
                    self.waypoints.guidance_law = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,0]
                    self.waypoints.waypoint_list_z = [self.ned_z,self.upz] 
                    self.desired(self.waypoints)   
            elif(self.foundstate == 2):
               #move to left bin
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_y-self.lefty,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 2.1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.lefty]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 2.1):
               #stay find the best position to grab bin
               rospy.logwarn("Looking the best position to grab bin") 
               self.grab_bin(2.2) 
            elif(self.foundstate == 2.2):
                self.waypoints.guidance_law = 1
               #descend
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_z-self.downz,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 2.3
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,0]
                    self.waypoints.waypoint_list_z = [self.ned_z,self.downz]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 2.3):
                self.waypoints.guidance_law = 1
                #ascend
                _euc_distance = pow(pow(self.ned_x-self.samex,2)+pow(self.ned_z-self.upz,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 3
                    self.waypoints.guidance_law = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.samex]
                    self.waypoints.waypoint_list_y = [self.ned_y,0]
                    self.waypoints.waypoint_list_z = [self.ned_z,self.upz] 
                    self.desired(self.waypoints)   
            elif(self.foundstate == 3):
               #move to leave waypoint1
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
               #move to leave waypoint2
                _euc_distance = pow(pow(self.ned_x-self.leavewaypointx,2)+pow(self.ned_y-self.leavewaypointy,2),0.5)
                if(_euc_distance <0.35):
                    self.state = 6
                    self.waypoints.guidance_law = 0
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leavewaypointx]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leavewaypointy]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)  

    def binmission(self):
        self.waypoints.waypoint_list_length = 2
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
                rospy.loginfo("Binmission is activated")
                self.binmission()
            else:
                self.activated = False
            rate.sleep()
def main():
    rospy.init_node("gate_mission", anonymous=False)
    rate = rospy.Rate(20)
    gate_mission = BinMission()
    last_detection = []
    while not rospy.is_shutdown() and gate_mission.activated:
        if(gate_mission.state != 6):
            rospy.loginfo("Binmission is activated")
            gate_mission.binmission()
        else:
            gate_mission.results()
        rate.sleep()
    rospy.spin()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:     
        pass