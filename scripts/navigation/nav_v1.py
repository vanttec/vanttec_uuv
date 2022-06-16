#!/usr/bin/env python
# -- coding: utf-8 --

from cgitb import enable
from distutils.ccompiler import new_compiler
import math
from pickle import FALSE
import time
import numpy as np
from rosdep2 import RosdepLookup
from yaml import Mark
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String,Int16
from geometry_msgs.msg import Pose, PoseStamped,Point,Vector3
from vanttec_uuv.msg import GuidanceWaypoints, obj_detected_list, rotateAction, rotateGoal, walkAction, walkGoal, gotoAction, gotoGoal, clusters_center_list
from octomap_server.msg import oclustAction, oclustGoal, points_list
import actionlib
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import time
#from vanttec_uuv.msg import clusters_center_list


# Default values
RTURN90 = math.pi/2
LTURN90 = -math.pi/2
SWEEP_RIGHT = math.pi/4
SWEEP_LEFT = -math.pi/4
DISTANCE_AWAY_EXPLORE=3
WALKDIS = 6
EXPLORE_THRESHOLD_X =  3
EXPLORE_THRESHOLD_Y = 8
FILENAME = "/ws/src/octomap_mapping/octomap_server/scripts/inputpointclouds.csv"


# Publisher para tomar fotos

# Explorar un punto, es importante orbitar para de esta manera detectar yolo


class uuv_instance:
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
        self.side = "police"
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

        self.oldclusters=set()
        self.newclusters=set()
        self.explored_zones=set()
        self.explore_threshold=3
        self.explore_thresholdx=pow(EXPLORE_THRESHOLD_X,2) #a
        self.explore_thresholdy=pow(EXPLORE_THRESHOLD_Y,2) #b
        self.newexploration=False
        self.markerArray = MarkerArray()
        self.auxmarker = Marker()
        self.auxmarker.header.frame_id = "world_ned"
        self.auxmarker.type = Marker.SPHERE
        self.auxmarker.action = Marker.ADD
        self.auxmarker.scale.x = EXPLORE_THRESHOLD_X
        self.auxmarker.scale.y = EXPLORE_THRESHOLD_Y
        self.auxmarker.scale.z = 0.4
        self.auxmarker.color.a = 0.3 # Don't forget to set the alpha!
        self.auxmarker.color.r = 1.0
        self.auxmarker.color.g = 0.0
        self.auxmarker.color.b = 0.0


        # ROS Subscribers
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        # rospy.Subscriber("/uuv_perception/buoy_1_pos_pub",Point,self.buoy1_callback)
        # rospy.Subscriber("/uuv_perception/buoy_2_pos_pub",Point,self.buoy2_callback)
        rospy.Subscriber("/markerwaypoint",Point,self.marker_callback)
        rospy.Subscriber('/uuv_perception/yolo_zed/objects_detected', obj_detected_list, self.detected_objects_callback)

        '''
        rospy.Subscriber("/usv_perception/yolo_zed/objects_detected", obj_detected_list, self.objs_callback)
        '''

        # ROS Publishers
        self.uuv_waypoints = rospy.Publisher("/uuv_guidance/guidance_controller/waypoints", GuidanceWaypoints, queue_size=10)
        self.uuv_path_pub = rospy.Publisher("/uuv_planning/motion_planning/desired_path", Path, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)
        self.octocluster=rospy.Publisher("/oldclusters",points_list,queue_size=10)
        #rospy.Subscriber("buoy_1_pos_pub",Point,self.buoy_vision_callback)

        #Waypoint test instead of perception node
        #This array shall be modified with zed inputs of distance  
        rospy.Subscriber("clusters_center",clusters_center_list,self.cluster_callback)

    def order(self,ele):
        return pow(pow(ele[0]-self.ned_x,2)+pow(ele[1]-self.ned_y,2),0.5)
    def analyze_cluster(self,unknown_cluster):
        newclusters=set()
        enter=0
        unknown_cluster1=unknown_cluster
        oldcluster1=self.oldclusters
        newold_cluster=set()
        while enter==0:
            for i in unknown_cluster1:
                #Far distance from sub/body
                if len(self.oldclusters)==0:
                    rospy.logwarn("Primera vez")
                    rospy.loginfo(i)
                    newclusters.add(i)
                    self.oldclusters.add(i)
                    sorted(newclusters,key=self.order)
                    self.newexploration=True
                    self.auxmarker.id = 0
                    self.auxmarker.pose.position.x = i[0]
                    self.auxmarker.pose.position.y = i[1]
                    self.auxmarker.pose.position.z = i[2]
                    self.auxmarker.pose.orientation.x = 0
                    self.auxmarker.pose.orientation.y = 0
                    self.auxmarker.pose.orientation.z = 0
                    self.auxmarker.pose.orientation.w = 1

                    self.markerArray.markers.append(self.auxmarker)
                    rospy.logwarn("Paso el clustermarker pub")
                else:
                    #Far distance from explored zones
                    rospy.logwarn("Old clusters")
                    rospy.logwarn(self.oldclusters)
                    for j in oldcluster1:
                        _euc_distance2=pow(i[0]-j[0],2)/self.explore_thresholdx+pow(i[1]-j[1],2)/self.explore_thresholdy
                        rospy.logwarn(_euc_distance2)
                        if _euc_distance2<1:
                            self.newexploration=False
                            break
                        else:
                            self.newexploration=True
                    if self.newexploration:
                        rospy.logwarn("Explored zones")
                        rospy.loginfo(_euc_distance2)
                        rospy.loginfo(i)
                        newold_cluster.add(i)
                        newclusters.add(i)
                        
                        self.auxmarker.id = len(self.oldclusters) +len(newold_cluster)+ 1
                        self.auxmarker.pose.position.x = i[0]
                        self.auxmarker.pose.position.y = i[1]
                        self.auxmarker.pose.position.z = i[2]
                        self.auxmarker.pose.orientation.x = 0
                        self.auxmarker.pose.orientation.y = 0
                        self.auxmarker.pose.orientation.z = 0
                        self.auxmarker.pose.orientation.w = 1
                        self.markerArray.markers.append(self.auxmarker)

                        sorted(newclusters,key=self.order)

                enter=1
        if len(newold_cluster)!=0:
            for index,value in enumerate(newold_cluster):
                self.oldclusters.add(value)
                rospy.logwarn("Paso el clustermarker pub")
        try:
            m_octocluster=points_list()
            v_octcluster=Vector3()
            m_octocluster.lenpoints=len(self.oldclusters)
            for i in newclusters:
                v_octcluster.x=i[0]
                v_octcluster.y=i[1]
                v_octcluster.z=i[2]
                m_octocluster.points.append(v_octcluster)
            self.octocluster.publish(m_octocluster)
        except Exception as e:
            rospy.logwarn(e)
        return newclusters



    def cluster_callback(self,msg):
        octocluster=[]
        for i in msg.clusters:
            rospy.logwarn(i)
            octocluster.append([i.x,i.y,i.z])
        cluster=[tuple(y) for y in octocluster]
        setCluster=set(cluster)
        newclusters =  self.analyze_cluster(setCluster)
        if  len(newclusters)!=0:
            self.newclusters=newclusters
            rospy.logwarn("New cluster")
            rospy.logwarn(self.newclusters)
            




    def detected_objects_callback(self, msg):
        self.objects_list = msg
        for object in msg.objects:
            if   object.clase == "gun":
                self.gun_x_coord = object.x
                self.buoy1_class_found = 1
            elif object.clase == "badge":
                self.badge_x_coord = object.x
                self.buoy2_class_found = 1

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
            if(self.buoy1.x!=0.0 and self.buoy2.x !=0.0):
                if(self.side == "police"):
                    if self.badge_x_coord < self.gun_x_coord:
                        self.foundimage = {
                        'X': self.buoy1.z - self.camera_offset_x,
                        'Y': self.buoy1.x,
                        'Z': self.buoy1.y - self.camera_offset_z
                        }
                    else:
                        self.foundimage = {
                        'X': self.buoy2.z - self.camera_offset_x,
                        'Y': self.buoy2.x,
                        'Z': self.buoy2.y - self.camera_offset_z
                        } 
                else:
                    if self.badge_x_coord < self.gun_x_coord:
                        self.foundimage = {
                        'X': self.buoy2.z - self.camera_offset_x,
                        'Y': self.buoy2.x,
                        'Z': self.buoy2.y - self.camera_offset_z
                        }
                    else:
                        self.foundimage = {
                        'X': self.buoy1.z - self.camera_offset_x,
                        'Y': self.buoy1.x,
                        'Z': self.buoy1.y - self.camera_offset_z
                        }
                self.movetobuoy = True

            # if self.side == "police":
            #     if self.badge_x_coord < self.gun_x_coord:
            #         self.buoy = self.right_buoy
            #         self.buoy_angle_body = self.right_buoy_angle_body
            #     else:
            #         self.buoy = self.left_buoy
            #         self.buoy_angle_body = self.left_buoy_angle_body
            # else:
            #     if self.badge_x_coord < self.gun_x_coord:
            #         self.buoy = self.left_buoy
            #         self.buoy_angle_body = self.left_buoy_angle_body
            #     else:
            #         self.buoy = self.right_buoy
            #         self.buoy_angle_body = self.right_buoy_angle_body

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
        rospy.loginfo("search")
        if(self.locatemarker == False):
            rospy.logwarn("Pahtmarker is not located")
            if self.searchstate == -1:
                #sweep to find 
                self.sweep(0)
                rospy.loginfo("sweeping")
            elif self.searchstate == 0:
                self.waypoints.guidance_law = 1
                #move 3 meter
                rospy.loginfo("Moving")
                _euc_distance = pow(pow(self.ned_x-self.searchx,2)+pow(self.ned_y-self.searchy,2),0.5)
                if(_euc_distance <0.35):
                    rospy.loginfo(_euc_distance)
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
        # rospy.logwarn("Desired sended")
        
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

class uuv_nav:
    def __init__(self):

        self.uuv = uuv_instance()
        self.isrot = False
        self.ismov = False
        self.clustered = None
        self.waypointstatus = None
        self.waypointtarget = None
        self.rot_control = 0 
        self.search_control = 0
        self.rot_client = actionlib.SimpleActionClient('rotate', rotateAction)
        self.rot_goal = rotateGoal()
        self.walk_client = actionlib.SimpleActionClient('walk', walkAction)
        self.walk_goal = walkGoal()
        self.goto_client = actionlib.SimpleActionClient('goto', gotoAction)
        self.goto_goal = gotoGoal()
        self.octoclient = actionlib.SimpleActionClient('oclust', oclustAction)
        self.octogoal = oclustGoal()

        
        self.search_step = 0
        self.iteration = 1
        rospy.loginfo("Waiting for rotate server")
        self.rot_client.wait_for_server()
        rospy.loginfo("Waiting for walk server")
        self.walk_client.wait_for_server()
        rospy.loginfo("Waiting for goto server")
        self.oclient = actionlib.SimpleActionClient('goto', gotoAction)
        self.oclient.wait_for_server()
        self.ggoal = gotoGoal()
        rospy.loginfo("Waiting for oclust server")
        self.octoclient.wait_for_server()
        self.nav_matrix = []
        self.enableoctomap = rospy.Publisher("/enablepcl",String, queue_size=10)
        self.enableclustering = rospy.Publisher("/start_clustering",String, queue_size=10)
        self.enableanalyseocto = rospy.Publisher("/readyforcluster",String,queue_size=10)
        self.eraseoctomap = rospy.Publisher("/erasemap",String, queue_size=10)
        self.pointlist_pub = rospy.Publisher("/octomap/point_list",points_list, queue_size=10)
        self.waiting=True
        self.size_octoarray=0
        self.prev_size_octoarray=0

        self.init_pose = Point()
        self.current_pose = Point()
        self.point_handler = Point()
        self.alg_control = 0
        # self.nav_matrix = [[0,0,0],
        #                    [0,0,0],
        #                    [0,0,0]]
        rospy.Subscriber('/enablepcl', String, self.octomap_waiting_callback)
        rospy.Subscriber('/arraylengthocto', Int16, self.arraylength_octo_callback)
        self.waiting_octo="Deactivate"
    def octomap_waiting_callback(self,msg):
        self.waiting_octo=msg.data
    def arraylength_octo_callback(self,msg):
        self.size_octoarray=msg.data

    def main(self):
        rate = rospy.Rate(10)
        print(rospy.get_time())
        uuv = self.uuv
        self.init_pose.x = uuv.ned_x
        self.init_pose.y = uuv.ned_y


        markerpub = rospy.Publisher('Clusters', MarkerArray,queue_size=1)
        rospy.logwarn("Published")


        while not rospy.is_shutdown():
            
            markerpub.publish(uuv.markerArray)
            if uuv.newexploration:
                rospy.logwarn("Explore")
                self.explore_cluster(uuv.newclusters,uuv)
            else:
                rospy.logwarn("Search")
                self.search(uuv)
            # self.walk(uuv, 3)
            # self.rotate(uuv, math.pi/2)
            # self.rotate(uuv,math.pi/2)
            rate.sleep()
            # rospy.spin()

    def search(self, uuv):
        #look subscriber of pathmarker
        rospy.loginfo("search")

        # walk actions should e be changed for a dynamic iterative option
        # self.search_control = self.check_cluster()

        # Also a good idea would be store the last pose 

        # Three ways of use, if 
        self.current_pose.x = self.uuv.ned_x
        self.current_pose.y = self.uuv.ned_y
        self.current_pose.z = self.uuv.ned_z

        if (self.search_control == 0):
            rospy.logwarn(self.search_step)
            if self.search_step == 0:
                
                # self.allign()
            
                if self.waiting_octo=="Deactivate":
                    self.walk()    
                    self.point_handler.x = self.uuv.ned_x
                    self.point_handler.y = self.uuv.ned_y
                    self.point_handler.z = self.uuv.ned_z + 1
                    # rospy.logwarn(self.point_handler)
                    self.goto(self.point_handler)
                    self.allign()
                    self.enableoctomap.publish("Activate")
                   
                else :

                    self.point_handler.x = self.uuv.ned_x
                    self.point_handler.y = self.uuv.ned_y
                    self.point_handler.z = self.uuv.ned_z - 1
                    self.goto(self.point_handler)
                    if self.size_octoarray==self.prev_size_octoarray :
                        rospy.sleep(7)
                        self.prev_size_octoarray=self.size_octoarray
                    ogoal=oclustGoal()
                    self.octoclient.send_goal(ogoal)
                    self.octoclient.wait_for_result()
                    rospy.sleep(2)
                    self.enableoctomap.publish("Deactivate") 
                    rospy.sleep(4)     
                    self.search_step = 0.1
                    
                # Capture

                #Detente
                #De frente ,Tomo la foto 
                #Deactivo 
                #Otro search

                #
            elif self.search_step == 0.1:
                if self.waiting_octo=="Deactivate":
                    self.search_step=1
                
            elif self.search_step == 1:
                
                
                if self.waiting_octo=="Deactivate":
                    self.rotate()
                    self.walk()
                    # Capture
                    self.rotate(LTURN90)
                    self.lallign()
                    self.point_handler.x = self.uuv.ned_x
                    self.point_handler.y = self.uuv.ned_y
                    self.point_handler.z = self.uuv.ned_z + 1
                    self.goto(self.point_handler)
                    # rospy.logwarn("ABAJO")
                    self.enableoctomap.publish("Activate")
                else:
                    if self.size_octoarray==self.prev_size_octoarray :
                        rospy.sleep(7)
                        self.prev_size_octoarray=self.size_octoarray
                    self.point_handler.x = self.uuv.ned_x
                    self.point_handler.y = self.uuv.ned_y
                    self.point_handler.z = self.uuv.ned_z - 1
                    self.goto(self.point_handler)
                    self.enableoctomap.publish("Deactivate")   
                    rospy.sleep(4)   
                    ogoal=oclustGoal()
                    self.octoclient.send_goal(ogoal)
                    self.octoclient.wait_for_result()
                    rospy.sleep(2)
                    # rospy.sleep(3)
                    self.search_step = 1.1
            elif self.search_step == 1.1:
                if self.waiting_octo=="Deactivate":
                    self.search_step=2
            elif self.search_step == 2:
                if self.waiting_octo=="Deactivate":
                    self.rotate(LTURN90)
                    self.walk()
                    self.walk()
                    # self.allign()
                    self.rotate()
                    #Capture
                    self.allign()
                    self.point_handler.x = self.uuv.ned_x
                    self.point_handler.y = self.uuv.ned_y
                    self.point_handler.z = self.uuv.ned_z + 1
                    self.goto(self.point_handler)
                    self.enableoctomap.publish("Activate")
                else:
                    if self.size_octoarray==self.prev_size_octoarray :
                        rospy.sleep(7)
                        self.prev_size_octoarray=self.size_octoarray  
                    self.point_handler.x = self.uuv.ned_x
                    self.point_handler.y = self.uuv.ned_y
                    self.point_handler.z = self.uuv.ned_z - 1
                    self.goto(self.point_handler)
                    ogoal=oclustGoal()
                    self.octoclient.send_goal(ogoal)
                    self.octoclient.wait_for_result()
                    rospy.sleep(2)
                    self.enableoctomap.publish("Deactivate")   
                    rospy.sleep(4)    
                    # rospy.sleep(3)  
                    self.search_step = 2.1
            elif self.search_step == 2.1:
                if self.waiting_octo=="Deactivate":
                    self.search_step=3
            elif self.search_step == 3:
                self.rotate()
                self.walk()
                self.rotate(LTURN90)
                self.allign()
                self.search_step = 0


            # elif self.search_step == 4:
            #     # Capture
            #     if uuv.waiting_octo=="Activate":
            #         self.enableclustering.publish("Deactivate")
            #         self.enableoctomap.publish("Deactivate")       
            #         self.enableanalyseocto.publish("Deactivate")      
            #         rospy.sleep(3)
            #         self.walk()
            #         self.search_step = 0
            #     elif uuv.waiting_octo =="Deactivate":
            #         self.walk()
            #         self.rotate(LTURN90)
            #         self.enableoctomap.publish("Activate")
            #         self.enableclustering.publish("Activate")


            # elif self.search_step == 5:
            #     self.alg_control = self.alg_control + 1
            #     #Check whats the best way to reset
            #     self.current_pose.x = uuv.ned_x
            #     self.current_pose.y = uuv.ned_y
            #     algx = [WALKDIS*3,  WALKDIS*3 , WALKDIS*3 , -WALKDIS*3, -WALKDIS*3 ,-WALKDIS*3 , WALKDIS*3 , WALKDIS*3]
            #     algy = [0        ,  0         , 0         , 0         , 0          ,0          ,0          , 0        ]
            #     # algx = [WALKDIS*3,  WALKDIS*3 , 0         , -WALKDIS*3, -WALKDIS*3 ,-WALKDIS*3 ,0          , WALKDIS*3]
            #     # algy = [0        ,  WALKDIS*3 , WALKDIS*3,  WALKDIS*3, 0          ,-WALKDIS*3 ,-WALKDIS*3 ,  -WALKDIS*3]
            #     #       "Frente     Derecha     Atras       Atras       Izquierda   Izquierda   Frente  Frente"
            #     # aux = self.alg_control // len(algx)
            #     newpoint = Point()
            #     newpoint.x = self.init_pose.x + algx[self.alg_control%len(algx)] + ((algx[self.alg_control%len(algx)]) * (self.alg_control//len(algx)))
            #     newpoint.y = self.init_pose.y + algy[self.alg_control%len(algy)] + ((algy[self.alg_control%len(algy)]) * (self.alg_control//len(algy)))
            #     # newpoint.x = self.init_pose.x + algx[self.alg_control%len(algx)] 
            #     # newpoint.y = self.init_pose.y + algy[self.alg_control%len(algy)] 
            #     newpoint.z = 0
            #     rospy.logwarn(newpoint)
            #     rospy.loginfo("Iteration ")
            #     rospy.loginfo (self.alg_control)
            #     # time.sleep(4)
            #     rospy.logwarn("step5")
            #     self.search_step = 0
    
        
        if self.search_control == 1: #clustered
            pass
            
    def check_detections(self):
        
        rospy.Subscriber("/cluster_topic", Point, self.clustered)
        rospy.Subscriber('/uuv_perception/yolo_zed/objects_detected', obj_detected_list, self.detected_objects_callback)

        # Priorities are: 
        # If an image is detected it's there
        # If we have a good cluster we go there
        # Anything else we use the regular algorithm

        if self.clustered <0.5:
            return 0
        elif self.clustered >=0.5:
            return 1
        
        # Implement yolo 
            # return a 2

    def rotate(self, rotation = RTURN90):
        self.rot_goal.goal_angle = rotation
        self.rot_client.send_goal(self.rot_goal)
        self.rot_client.wait_for_result()
        rospy.sleep(1)
        

    def walk(self, walk_dis = WALKDIS):
        self.walk_goal.walk_dis = walk_dis
        self.walk_client.send_goal(self.walk_goal)
        self.walk_client.wait_for_result()
        rospy.sleep(1)

    def goto(self, point):
        self.goto_goal.goto_point = point
        self.goto_client.send_goal(self.goto_goal)
        self.goto_client.wait_for_result()

    def allign(self):
        while abs(self.uuv.yaw) > 0.07:
            # rospy.loginfo(self.uuv.yaw)
            self.rot_goal.goal_angle = RTURN90/90
            self.rot_client.send_goal(self.rot_goal)
            self.rot_client.wait_for_result()

    def lallign(self):
        while abs(self.uuv.yaw) > 0.07:
            # rospy.loginfo(self.uuv.yaw)
            self.rot_goal.goal_angle = LTURN90/90
            self.rot_client.send_goal(self.rot_goal)
            self.rot_client.wait_for_result()


    def orbit(self, point, walk_dis = WALKDIS):
        #First we go to a point to orbit
        rospy.loginfo("Goingto")
        point.x = point.x - walk_dis

        self.goto(point)
        time.sleep(1)
        while abs(self.uuv.yaw) > 0.1:
            rospy.loginfo(self.uuv.yaw)
            self.rotate(RTURN90/90)
        self.rotate(LTURN90)
        for i in range(2):
            rospy.loginfo("supose first rot")
            self.rotate()

        rospy.logwarn("GONE 1")
        point.x = point.x + walk_dis
        point.y = point.y + walk_dis
        self.goto(point)
        for i in range(2):
            rospy.loginfo("supose sec rot")
            self.rotate(LTURN90)
        rospy.logwarn("GONE 2")
        point.x = point.x + walk_dis
        point.y = point.y - walk_dis
        self.goto(point)
        for i in range(2):
            self.rotate(LTURN90)
            rospy.loginfo("supose thr rot")

        rospy.logwarn("GONE 3")
        point.x = point.x - walk_dis
        point.y = point.y - walk_dis
        self.goto(point)
        self.rotate(LTURN90)
        rospy.logwarn("GONE 4")
        point.x = point.x - walk_dis
        point.y = point.y + walk_dis
        self.goto(point)
        rospy.logwarn("GONE 5")
    def sweep(self):
        self.rotate(SWEEP_RIGHT)
        rospy.sleep(4)
        self.rotate(SWEEP_LEFT)
        rospy.sleep(4)
        self.rotate(SWEEP_LEFT)
        rospy.sleep(4)
        self.rotate(SWEEP_RIGHT)


    def explore_cluster(self,newcluster,uuv):

        self.current_pose.x = self.uuv.ned_x
        self.current_pose.y = self.uuv.ned_y
        self.current_pose.z = self.uuv.ned_z
        f = lambda x:list(x)
        exploration_coord = [f(x) for x in newcluster]
        rospy.logwarn(exploration_coord)
        self.enableoctomap.publish("Deactivate")
        if self.waiting_octo=="Deactivate":
            for i in exploration_coord:
                self.ggoal.goto_point.x = i[0]-DISTANCE_AWAY_EXPLORE
                self.ggoal.goto_point.y = -i[1]
                self.ggoal.goto_point.z = uuv.ned_z
                self.oclient.send_goal(self.ggoal)
                self.oclient.wait_for_result()
                self.sweep()
            #self.eraseoctomap.publish("Activate")
            rospy.sleep(5)
            f = open('/ws/src/octomap_mapping/octomap_server/scripts/inputpointclouds.csv', "w+")
            f.truncate()
            f.close()
            # self.ggoal.goto_point.x =uuv.ned_x
            # self.ggoal.goto_point.y = self.current_pose.y
            # self.ggoal.goto_point.z = self.current_pose.z
            # self.oclient.send_goal(self.ggoal)
            # self.oclient.wait_for_result()
            # self.walk_client.wait_for_result()
            v_pointList = points_list()
            v_pointList.points=[]
            self.pointlist_pub.publish(v_pointList)
            rospy.logwarn("Finish exploration")
            # self.eraseoctomap.publish("Activate")
            
            # rospy.sleep(5)
            # self.eraseoctomap.publish("Deactivate")
            self.allign()
            uuv.newexploration=False
            self.search_step = 0

        

    # def rotate(self):
    #     self.rot_goal.goal_angle = RTURN90
    #     self.rot_client.send_goal(self.rot_goal)
    #     self.rot_client.wait_for_result()
    #     time.sleep(1)


    
    def create_nav_mat(self,size):
        for i in range(size):
            aux = aux.append(0)
            for j in range(size):
                self.nav_matrix
        

def cluster_marker(name,pose,id):
    marker = Marker()
    marker.header.frame_id = "world_ned"
    marker.id = id
    marker.header.stamp = rospy.get_rostime()

    pub = rospy.Publisher(name, Marker, queue_size=10)
    marker.type = marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = EXPLORE_THRESHOLD_Y
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 0.8 # Don't forget to set the alpha!
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    pub.publish(marker)
    rospy.logwarn("Published")


     


if __name__ == "__main__":
    try:
        rospy.init_node("navigation", anonymous=False)
        i = uuv_nav()
        i.main()
    except rospy.ROSInterruptException:     
        pass
