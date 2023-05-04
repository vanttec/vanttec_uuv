#! /usr/bin/env python3
# import roslib
# roslib.load_manifest('vanttec_uuv')
import rospy
import actionlib
import time
from geometry_msgs.msg import Pose, Point
from vanttec_uuv.msg import gotoAction
from nav_v1 import uuv_instance
import math

class gotoServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('goto',gotoAction, self.execute, False)
        self.server.start()
        self.isrot = False
        self.ismov = False
        self.ned_x = None
        self.ned_y = None
        self.ned_z = None
        self.yaw = None
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        self.uuv = uuv_instance()
        self.waypointtarget = None        

        self.goto_control = -1 
        self.uuv = uuv_instance()
        rospy.logwarn(self.uuv.waypoints.heading_setpoint )
        rospy.loginfo("goto server loaded OK")


    def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
        uuv = self.uuv
        print(goal)
        r = rospy.Rate(10)
        uuv.waypoints.guidance_law = 1
        #move 3 meter
        rospy.loginfo("Moving")
        #rospy.logwarn(goal.goto_point)
        print(self.yaw)
        uuv.searchx = goal.goto_point.x
        uuv.searchy = goal.goto_point.y
        uuv.searchz = goal.goto_point.z
        #rospy.loginfo(uuv.searchx - uuv.ned_x)
        #rospy.loginfo(uuv.searchy - uuv.ned_y)
        
        _euc_distance = pow(pow(uuv.ned_x-uuv.searchx,2)+pow(uuv.ned_y-uuv.searchy,2)+pow(uuv.ned_z-uuv.searchz,2),0.5)
        rospy.loginfo(_euc_distance)
        self.th = 0.1
        while _euc_distance >= self.th:
            _euc_distance = pow(pow(uuv.ned_x-uuv.searchx,2)+pow(uuv.ned_y-uuv.searchy,2)+pow(uuv.ned_z-uuv.searchz,2),0.5)
            #rospy.logwarn(_euc_distance)
            # rospy.loginfo(_euc_distance)
            uuv.waypoints.waypoint_list_x = [uuv.ned_x,uuv.searchx]
            uuv.waypoints.waypoint_list_y = [uuv.ned_y,uuv.searchy]
            uuv.waypoints.waypoint_list_z = [uuv.ned_z,uuv.searchz]   
            uuv.desired(uuv.waypoints)
            rospy.logwarn("IM here")
            r.sleep()
            if _euc_distance < 0.35:
                self.searchstate = -1
                self.goto_control  = 1
                break
            if self.goto_control == 1 :
                rospy.loginfo('Succeeded going')
        self.server.set_succeeded()


    
    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z  
        


if __name__ == '__main__':
    rospy.init_node('goto_server')
    server = gotoServer()
    rospy.spin()