#!/usr/bin/env python
# -- coding: utf-8 --

import math
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped,Point
from vanttec_uuv.msg import GuidanceWaypoints, obj_detected_list
from nav_msgs.msg import Path

# Class Definition
class BuoyMission:
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
        self.choose_side = 'police'
        self.distance_away = 5
        self.waypoints = GuidanceWaypoints()
        self.uuv_path = Path()
        self.heading_threshold = 0.01
        self.depth_threshold = 0.
        #findimage works as a counter however with vision input once it identifies the target it should set this variable to True
        self.findimage = -2
        self.searchx = 0.0
        self.searchy = 0.0
        self.searchz = 0.0
        self.sweepstate = -1
        self.foundstate = -2
        self.enterwaypoint = 0.0
        self.staywaypoint = 0.0
        self.ylabel = 0.0
        self.timewait=0.0
        self.buoy1 = Point()
        self.buoy2 = Point()
        self.buoy1.x = 0.0
        self.buoy1.y = 0.0
        self.buoy1.z = 0.0
        self.buoy2.x = 0.0
        self.buoy2.y = 0.0
        self.buoy2.z = 0.0
        self.foundimage = {}
        self.side = "LEFT"
        self.locatemarker = False
        self.findimage = False
        self.movetobuoy = False
        #Waypoint test instead of perception node

        self.gun_x_coord = 0
        self.badge_x_coord = 0
        self.buoy1_class_found = 0
        self.buoy2_class_found = 0

        self.camera_offset_x = 0.8
        self.camera_offset_y = 0.0
        self.camera_offset_z = -0.1

        # ROS Subscribers
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        rospy.Subscriber("buoy_1_pos_pub",Point,self.bouy1_callback)
        rospy.Subscriber("buoy_2_pos_pub",Point,self.bouy2_callback)
        rospy.Subscriber("/markerwaypoint",Point,self.marker_callback)
        '''
        rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.objs_callback)
        '''

        # ROS Publishers
        self.uuv_waypoints = rospy.Publisher("/uuv_guidance/guidance_controller/waypoints", GuidanceWaypoints, queue_size=10)
        self.uuv_path_pub = rospy.Publisher("/uuv_planning/motion_planning/desired_path", Path, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)
        #rospy.Subscriber("buoy_1_pos_pub",Point,self.buoy_vision_callback)

        #Waypoint test instead of perception node
        #This array shall be modified with zed inputs of distance    

    def detected_objects_callback(self, msg):
        self.objects_list = msg
        for object in msg.objects:
            if   object.clase == "gun":
                self.gun_x_coord = object.x
                self.buoy1_class_found = 1
            elif object.clase == "badge":
                self.badge_x_coord = object.x
                self.buoy2_class_found = 1

    def buoy1_callback(self, msg):
        self.buoy1 = msg
        buoy = Point()
        if self.buoy1.x < self.buoy2.x:
            buoy = self.buoy2
            self.buoy2 = self.buoy1
            self.buoy1 = buoy

    def buoy2_callback(self, msg):
        self.buoy2 = msg
        buoy = Point()
        if self.buoy1.x < self.buoy2.x:
            buoy = self.buoy2
            self.buoy2 = self.buoy1
            self.buoy1 = buoy

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
        if(self.locatemarker == True):
            if(self.bouy1.x!=0.0 and self.bouy2.x !=0.0):
                if(self.side == "LEFT"):
                    self.foundimage = {
                    'X': self.bouy2.z,
                    'Y': self.bouy2.x,
                    'Z': self.bouy2.y
                    } 
                else:
                    self.foundimage = {
                    'X': self.bouy1.z,
                    'Y': self.bouy1.x,
                    'Z': self.bouy1.y
                    } 
                self.movetobuoy = True

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
                if(self.locatemarker == False):
                    self.searchx = self.ned_x + 2.5
                    self.searchy = self.ned_y + 0
                else:
                    self.searchx = self.markerx + self.differencemarkerx
                    self.searchy = self.markery + self.differencemarkery
                    if(self.movetobuoy == True):
                        self.findimage = True
                self.sweepstate = -1
                self.findimage += 1  
                self.desired(self.waypoints)

            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint -= math.pi/400.0
                self.waypoints.waypoint_list_x = [0, 0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0,0]
                self.desired(self.waypoints) 
    def touch_buoy(self,nextmission):
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
    def wait(self,nextmission):
        self.waypoints.guidance_law = 0
        self.waypoints.heading_setpoint = 0
        timeduration = rospy.get_time()-self.timewait
        rospy.logwarn("Analyzing image with yolo neural network")
        if(timeduration >= 3):
            self.timewait = 0
            rospy.logwarn("Found image")
            self.foundstate = nextmission           
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
        elif(self.findimage == False):
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
            if(self.foundstate== -2):
                rospy.logwarn("Analizing the image to choose side")
                self.wait(-1)
            elif(self.foundstate == -1):
                self.enterwaypoint = self.ned_x+self.foundimage['X']+1
                self.staywaypoint = self.ned_x+self.foundimage['X']+0.4
                self.ylabel = self.ned_y + self.foundimage['Y'] - 0.45
                self.leavewaypoint1x = self.staywaypoint 
                self.leavewaypoint1y = self.ned_y+self.foundimage['Y']+1
                self.leavewaypoint2x = self.ned_x+self.foundimage['X']+2
                self.leavewaypoint2y = self.leavewaypoint1y 
                self.leavewaypoint3y = self.ned_y + self.foundimage['Y']-1
                self.leavewaypoint3x = self.leavewaypoint2x
                self.leavewaypoint4y = self.leavewaypoint3y
                self.leavewaypoint4x = self.leavewaypoint3x+2
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
               #move to stay waypoint
                _euc_distance = pow(pow(self.ned_x-self.staywaypoint,2)+pow(self.ned_y-self.ylabel,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 2
                    self.waypoints.guidance_law = 0
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.staywaypoint]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.ylabel]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)     
            elif(self.foundstate == 2):
               #stay until buoymission is completed
               rospy.logwarn("Looking the best position to touch buoy") 
               self.touch_buoy(3)
            #    self.foundstate = 3
            elif(self.foundstate == 3):
                self.waypoints.guidance_law = 1
                rospy.logwarn("leave1")
               #move to leave waypoint1
                _euc_distance = pow(pow(self.ned_x-self.leavewaypoint1x,2)+pow(self.ned_y-self.leavewaypoint1y,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 4
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leavewaypoint1x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leavewaypoint1y]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 4):
                rospy.logwarn("leave2")
               #move to leave waypoint2
                _euc_distance = pow(pow(self.ned_x-self.leavewaypoint2x,2)+pow(self.ned_y-self.leavewaypoint2y,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 5
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leavewaypoint2x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leavewaypoint2y]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 5):
               #move to leave waypoint3
                rospy.logwarn("leave3")
                _euc_distance = pow(pow(self.ned_x-self.leavewaypoint3x,2)+pow(self.ned_y-self.leavewaypoint3y,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 6
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leavewaypoint3x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leavewaypoint3y]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.foundstate == 6):
                rospy.logwarn("leave4")
               #move to leave waypoint4
                _euc_distance = pow(pow(self.ned_x-self.leavewaypoint4x,2)+pow(self.ned_y-self.leavewaypoint4y,2),0.5)
                if(_euc_distance <0.35):
                    self.foundstate = 7
                    self.waypoints.guidance_law = 0
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leavewaypoint4x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leavewaypoint4y]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   

    def buoymission(self):
        self.waypoints.waypoint_list_length = 2
        self.waypoints.guidance_law = 1
        #self.waypoints.depth_setpoint = 4
        #depth_error = self.ned_z - self.waypoints.depth_setpoint
        
        if(self.state == -1):
            self.search()

    def desired(self, path):
        self.uuv_waypoints.publish(path)
        self.uuv_path.header.stamp = rospy.Time.now()
        self.uuv_path.header.frame_id = "world_ned"
        del self.uuv_path.poses[:]
        for index in range(path.waypoint_list_length):
            pose = PoseStamped()
            pose.header.stamp       = rospy.Time.now()
            pose.header.frame_id    = "world_ned"
            pose.pose.position.x    = path.waypoint_list_x[index]
            pose.pose.position.y    = path.waypoint_list_y[index]
            pose.pose.position.z    = path.waypoint_list_z[index]
            self.uuv_path.poses.append(pose)
        self.uuv_path_pub.publish(self.uuv_path)
    def results(self):
        rospy.logwarn("Buoy mission finished")
      



    def activate(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.activated:
            if(self.state != 7):
                #rospy.loginfo("Buoymission is activated")
                self.buoymission()
            else:
                self.activated = False
            rate.sleep()
def main():
    rospy.init_node("buoy_mission", anonymous=False)
    rate = rospy.Rate(20)
    buoy_mission = BuoyMission()
    last_detection = []
    while not rospy.is_shutdown() and buoy_mission.activated:
        if(buoy_mission.state != 7):
            #rospy.loginfo("Buoymission is activated")
            buoy_mission.buoymission()
        else:
            buoy_mission.results()
        rate.sleep()
    rospy.spin()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:     
        pass