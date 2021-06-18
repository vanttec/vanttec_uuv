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
        self.foundstate = -2
        self.enterwaypoint = 0.0
        self.leavewaypoint = 0.0
        self.ylabel = 0.0
        self.gateposition = Point()
        self.finalgateposition = Point()
        self.gateposition.z = 0.0
        self.timewait=0.0
        self.analizegatex = 0.0
        self.analizegatey = 0.0
        self.cameraoffsetx = 0.8
        #self.cameraoffsety = 0.0
        self.cameraoffsetz = -0.1
        #Waypoint test instead of perception node


        # ROS Subscribers
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        rospy.Subscriber("gate_left_waypoint",Point,self.gate_vision_callback)
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
    def gate_vision_callback(self,msg):
        self.gateposition.x = msg.x + self.cameraoffsetx
        self.gateposition.y = msg.y #+ self.cameraoffsety
        self.gateposition.z = msg.z + self.cameraoffsetz
        
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
    def wait(self,nextmission):
        self.waypoints.guidance_law = 0
        self.waypoints.heading_setpoint = 0
        timeduration = rospy.get_time()-self.timewait
        #rospy.logwarn("Camera position")
        #rospy.logwarn(self.gateposition.x)
        #rospy.logwarn(self.gateposition.y)
        #rospy.logwarn(self.gateposition.z)
        self.finalgateposition.x = self.gateposition.z
        self.finalgateposition.y = self.gateposition.x
        self.finalgateposition.z = self.gateposition.y
        if(timeduration >= 2):
            self.timewait = 0
            self.foundstate = nextmission
           
    def search(self):
        #look subscriber of image distance
        #if(self.findimage <= 2):
        if(self.gateposition.z ==0.0):
            rospy.logwarn("Searching image")
            if self.searchstate == -1:
                #sweep to find 
                self.sweep(0)
            elif self.searchstate ==0:
                self.waypoints.guidance_law = 1
                #move 2 meter
                _euc_distance = pow(pow(self.ned_x-self.searchx,2)+pow(self.ned_y-self.searchy,2),0.5)
                if(_euc_distance <0.35):
                    #self.findimage += 1  
                    self.searchstate = -1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.searchx]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.searchy]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
        else:
            if(self.foundstate == -2):
                #Analize the image to choose side
                rospy.logwarn("Analizing the image to choose side")
                self.wait(-1)
            elif(self.foundstate == -1):
                rospy.logwarn("Chose right side")
                self.enterwaypoint = self.ned_x+self.finalgateposition.x-1
                self.leavewaypoint = self.ned_x+self.finalgateposition.x+2
                self.ylabel = self.ned_y + self.finalgateposition.y 
                self.leavewaypointx2 = self.ned_x+self.finalgateposition.x+5
                self.leavewaypointy2 = self.ned_y + self.finalgateposition.y-1
                self.foundstate = 0
            elif(self.foundstate == 0):
                self.waypoints.guidance_law = 1
                #move to enter waypoint
                _euc_distance = pow(pow(self.ned_x-self.enterwaypoint,2)+pow(self.ned_y-self.ylabel,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.enterwaypoint]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.ylabel]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)
            elif(self.foundstate == 1):
                self.waypoints.guidance_law = 1
               #move to leave waypoint1
                _euc_distance = pow(pow(self.ned_x-self.leavewaypoint,2)+pow(self.ned_y-self.ylabel,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 2
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leavewaypoint]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.ylabel]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)     
            elif(self.foundstate == 2):
                self.waypoints.guidance_law = 1
               #move to leave waypoint2
                _euc_distance = pow(pow(self.ned_x-self.leavewaypointx2,2)+pow(self.ned_y-self.leavewaypointy2,2),0.5)
                if(_euc_distance <0.35):
                    self.state = 6
                    self.waypoints.guidance_law = 0
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leavewaypointx2]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leavewaypointy2]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)  

    def gatemission(self):
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
        rospy.loginfo("Gate mission finished");   



    def activate(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.activated:
            rospy.loginfo(self.state ) 
            if(self.state != 6):
                #rospy.loginfo("Gatemission is activated")
                self.gatemission()
            else:
                self.activated = False
            rate.sleep()
def main():
    rospy.init_node("gate_mission", anonymous=False)
    rate = rospy.Rate(20)
    gate_mission = GateMission()
    last_detection = []
    while not rospy.is_shutdown() and gate_mission.activated:
        if(gate_mission.state != 6):
            #rospy.loginfo("Gatemission is activated")
            gate_mission.gatemission()
        else:
            gate_mission.results()
        rate.sleep()
    rospy.spin()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
