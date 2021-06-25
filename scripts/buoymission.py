#!/usr/bin/env python
# -- coding: utf-8 --

import math
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped, Point
from vanttec_uuv.msg import GuidanceWaypoints, obj_detected_list, DetectedObstacles
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
        self.search_state = -1
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
        #find_image works as a counter however with vision input once it identifies the target it should set this variable to True
        self.find_image = -2
        self.search_x = 0.0
        self.search_y = 0.0
        self.searchz = 0.0
        self.sweep_state = -1
        self.found_state = -2
        self.enter_waypoint = 0.0
        self.stay_waypoint = 0.0
        self.y_label = 0.0
        self.time_wait=0.0
        self.bouy_1 = Point()
        self.bouy_2 = Point()
        self.bouy_1.x = 0.0
        self.bouy_1.y = 0.0
        self.bouy_1.z = 0.0
        self.bouy_2.x = 0.0
        self.bouy_2.y = 0.0
        self.bouy_2.z = 0.0
        self.found_image = {}
        self.side = "gangster"
        self.locatemarker = False
        self.find_image = False
        self.movetobuoy = False

        self.gun_x_coord = 0
        self.badge_x_coord = 0

        # ROS Subscribers
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        rospy.Subscriber('/uuv_perception/yolo_zed/objects_detected', obj_detected_list, self.detected_objects_callback)
        # rospy.Subscriber("/uuv_perception/buoy_1_pos_pub",Point,self.bouy1_callback)
        # rospy.Subscriber("/uuv_perception/buoy_2_pos_pub",Point,self.bouy2_callback)
        rospy.Subscriber("/uuv_perception/buoys_position",DetectedObstacles,self.bouys_callback)
        rospy.Subscriber("/markerwaypoint",Point,self.marker_callback)

        # ROS Publishers
        self.uuv_waypoints = rospy.Publisher("/uuv_guidance/guidance_controller/waypoints", GuidanceWaypoints, queue_size=10)
        self.uuv_path_pub = rospy.Publisher("/uuv_planning/motion_planning/desired_path", Path, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)

    def marker_callback(self, msg):
        if(msg.x != 1 and msg.y != 0):
            self.callback_marker_x = msg.x
            self.callback_marker_y =  msg.y
            self.callback_marker_z = msg.z
            self.locatemarker = True

    def detected_objects_callback(self, msg):
        self.objects_list = msg
        for object in msg.objects:
            if   object.clase == "gun":
                self.gun_x_coord = object.x
            elif object.clase == "badge":
                self.badge_x_coord = object.x

    def bouys_callback(self, msg):
        for buoy in msg.obstacles:
            if buoy.type == "Buoy_1":
                self.bouy_1 = buoy.position
            else:
                self.bouy_2 = buoy.position

    # def bouy1_callback(self, msg):
    #     self.bouy_1 = msg

    # def bouy2_callback(self, msg):
    #     self.bouy_2 = msg

    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z     
         
    def sweep(self,next_mission):
        self.waypoints.guidance_law = 0
        if(self.bouy_1.x!=0.0 and self.bouy_2.x !=0.0):
            if self.side == "police":
                if self.badge_x_coord < self.badge_x_coord:
                    self.final_gate_position.x = self.left_gate_position.x
                    self.final_gate_position.y = self.left_gate_position.y
                    self.final_gate_position.z = self.left_gate_position.z
                else:
                    self.final_gate_position.x = self.right_gate_position.x
                    self.final_gate_position.y = self.right_gate_position.y
                    self.final_gate_position.z = self.right_gate_position.z
            else:
                if self.badge_x_coord < self.badge_x_coord:
                    self.final_gate_position.x = self.right_gate_position.x
                    self.final_gate_position.y = self.right_gate_position.y
                    self.final_gate_position.z = self.right_gate_position.z
                else:
                    self.final_gate_position.x = self.left_gate_position.x
                    self.final_gate_position.y = self.left_gate_position.y
                    self.final_gate_position.z = self.left_gate_position.z

            
        if(self.locatemarker == True):
            if(self.bouy1.x!=0.0 and self.bouy2.x !=0.0):
                if(self.side == "gun"):
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
        if(self.sweep_state == -1):
            if (self.waypoints.heading_setpoint <=  -math.pi/4):
                self.sweep_state = 2
            self.waypoints.heading_setpoint -= math.pi/400.0
            self.waypoints.waypoint_list_x = [0, 0]
            self.waypoints.waypoint_list_y = [0, 0]
            self.waypoints.waypoint_list_z = [0, 0]
            self.desired(self.waypoints)
        elif(self.sweep_state == 2): 
            if (self.waypoints.heading_setpoint >= math.pi/4):
                self.sweep_state = 2.1
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint += math.pi/400.0
                self.waypoints.waypoint_list_x = [0 ,0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0, 0]
                self.desired(self.waypoints)
        elif(self.sweep_state == 2.1): 
            if (self.waypoints.heading_setpoint <= 0):
                self.waypoints.guidance_law = 0
                self.search_state = next_mission
                if(self.locatemarker == False):
                    self.searchx = self.ned_x + 2.5
                    self.searchy = self.ned_y + 0
                else:
                    self.searchx = self.marker_x + self.difference_marker_x
                    self.searchy = self.marker_y + self.difference_marker_y
                    if(self.movetobuoy == True):
                        self.find_image = True
                self.sweep_state = -1
                self.find_image += 1  
                self.desired(self.waypoints)
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint -= math.pi/400.0
                self.waypoints.waypoint_list_x = [0, 0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0,0]
                self.desired(self.waypoints) 

    def touch_buoy(self,next_mission):
        self.waypoints.guidance_law = 0
        if(self.sweep_state == -1):
            if (self.waypoints.heading_setpoint <=  -math.pi/8):
                self.sweep_state = 2
            self.waypoints.heading_setpoint -= math.pi/400.0
            self.waypoints.waypoint_list_x = [0, 0]
            self.waypoints.waypoint_list_y = [0, 0]
            self.waypoints.waypoint_list_z = [0, 0]
            self.desired(self.waypoints)
        elif(self.sweep_state == 2): 
            if (self.waypoints.heading_setpoint >= math.pi/8):
                self.sweep_state = 2.1
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint += math.pi/400.0
                self.waypoints.waypoint_list_x = [0, 0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0, 0]
                self.desired(self.waypoints)
        elif(self.sweep_state == 2.1): 
            if (self.waypoints.heading_setpoint <= 0):
                self.waypoints.guidance_law = 0
                self.found_state = next_mission
                self.search_x = self.ned_x + 3
                self.search_y = self.ned_y
                self.sweep_state = -1
                self.desired(self.waypoints)
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint -= math.pi/400.0
                self.waypoints.waypoint_list_x = [0, 0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0, 0]
                self.desired(self.waypoints)

    def wait(self,next_mission):
        self.waypoints.guidance_law = 0
        self.waypoints.heading_setpoint = 0
        timeduration = rospy.get_time()-self.time_wait
        rospy.logwarn("Analyzing image with yolo neural network")
        if(timeduration >= 3):
            self.time_wait = 0
            rospy.logwarn("Found image")
            self.found_state = next_mission

    def search(self):
        #look subscriber of pathmarker
        if(self.locatemarker == False):
            rospy.logwarn("Pahtmarker is not located")
            if self.search_state == -1:
                #sweep to find 
                self.sweep(0)
            elif self.search_state == 0:
                self.waypoints.guidance_law = 1
                #move 3 meter
                euc_distance = pow(pow(self.ned_x-self.search_x,2)+pow(self.ned_y-self.search_y,2),0.5)
                if(euc_distance <0.35):
                    self.search_state = -1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.search_x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.search_y]
                    self.waypoints.waypoint_list_z = [0, 0]   
                    self.desired(self.waypoints)
                    #look subscriber of image distance    
        elif(self.find_image == False):
            rospy.logwarn("Pahtmarker is located")
            self.time_wait = rospy.get_time()
            if self.search_state == -1:
                self.marker_x = self.callback_marker_x
                self.marker_y = self.callback_marker_y
                self.marker_z = self.callback_marker_z
                self.difference_marker_x = self.callback_marker_x - self.ned_x
                self.difference_marker_y = self.callback_marker_y - self.ned_y
                self.difference_marker_z = self.callback_marker_z - self.ned_z
                self.search_state = 0
            if self.search_state == 0:
                self.waypoints.guidance_law = 1
                #move 3 meter
                _euc_distance = pow(pow(self.ned_x-self.marker_x,2)+pow(self.ned_y-self.marker_y,2),0.5)
                if(_euc_distance <0.35):
                    self.search_state = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.marker_x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.marker_y]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif self.search_state ==1: 
                #sweep to find 
                self.sweep(0)           
        else:
            if(self.found_state== -2):
                rospy.logwarn("Analizing the image to choose side")
                self.wait(-1)
            elif(self.found_state == -1):
                self.enter_waypoint = self.ned_x+self.found_image['X']-1
                self.stay_waypoint = self.ned_x+self.found_image['X']-0.3
                self.y_label = self.ned_y + self.found_image['Y']
                self.leave_waypoint_1x = self.stay_waypoint 
                self.leave_waypoint_1y = self.ned_y+self.found_image['Y']+1
                self.leave_waypoint_2x = self.ned_x+self.found_image['X']+2
                self.leave_waypoint_2y = self.leave_waypoint_1y 
                self.leave_waypoint_3y = self.ned_y + self.found_image['Y']-1
                self.leave_waypoint_3x = self.leave_waypoint_2x
                self.leave_waypoint_4y = self.leave_waypoint_3y
                self.leave_waypoint_4x = self.leave_waypoint_3x+2
                self.found_state = 0
            elif(self.found_state == 0):
                self.waypoints.guidance_law = 1
                #move to enter waypoint
                euc_distance = pow(pow(self.ned_x-self.enter_waypoint,2)+pow(self.ned_y-self.y_label,2),0.5)
                if(euc_distance <0.35):
                    self.found_state = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.enter_waypoint]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.y_label]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)
            elif(self.found_state == 1):
               #move to stay waypoint
                euc_distance = pow(pow(self.ned_x-self.stay_waypoint,2)+pow(self.ned_y-self.y_label,2),0.5)
                if(euc_distance <0.35):
                    self.found_state = 2
                    self.waypoints.guidance_law = 0
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.stay_waypoint]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.y_label]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)     
            elif(self.found_state == 2):
               #stay until bouymission is completed
               rospy.logwarn("Looking the best position to touch buoy") 
               self.touch_buoy(3)
            elif(self.found_state == 3):
                self.waypoints.guidance_law = 1
                rospy.logwarn("leave1")
               #move to leave waypoint1
                euc_distance = pow(pow(self.ned_x-self.leave_waypoint_1x,2)+pow(self.ned_y-self.leave_waypoint_1y,2),0.5)
                if(euc_distance <0.35):
                    self.found_state = 4
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leave_waypoint_1x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leave_waypoint_1y]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.found_state == 4):
                rospy.logwarn("leave2")
               #move to leave waypoint2
                euc_distance = pow(pow(self.ned_x-self.leave_waypoint_2x,2)+pow(self.ned_y-self.leave_waypoint_2y,2),0.5)
                if(euc_distance <0.35):
                    self.found_state = 5
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leave_waypoint_2x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leave_waypoint_2y]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.found_state == 5):
               #move to leave waypoint3
                rospy.logwarn("leave3")
                euc_distance = pow(pow(self.ned_x-self.leave_waypoint_3x,2)+pow(self.ned_y-self.leave_waypoint_3y,2),0.5)
                if(euc_distance <0.35):
                    self.found_state = 6
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leave_waypoint_3x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leave_waypoint_3y]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   
            elif(self.found_state == 6):
                rospy.logwarn("leave4")
               #move to leave waypoint4
                euc_distance = pow(pow(self.ned_x-self.leave_waypoint_4x,2)+pow(self.ned_y-self.leave_waypoint_4y,2),0.5)
                if(euc_distance <0.35):
                    self.found_state = 7
                    self.waypoints.guidance_law = 0
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leave_waypoint_4x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leave_waypoint_4y]
                    self.waypoints.waypoint_list_z = [0,0]   
                    self.desired(self.waypoints)   

    def buoy_mission(self):
        self.waypoints.waypoint_list_length = 2
        self.waypoints.guidance_law = 1
        
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
        rospy.logwarn("Buoy mission finished")

    def activate(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.activated:
            if(self.state != 7):
                rospy.loginfo("Buoy mission is activated")
                self.buoy_mission()
            else:
                self.activated = False
            rate.sleep()

def main():
    rospy.init_node("buoy_mission", anonymous=False)
    rate = rospy.Rate(20)
    buoy_mission = BuoyMission()
    while not rospy.is_shutdown() and buoy_mission.activated:
        if(buoy_mission.state != 7):
            rospy.loginfo("Buoy mission is activated")
            buoy_mission.buoy_mission()
        else:
            buoy_mission.results()
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:     
        pass