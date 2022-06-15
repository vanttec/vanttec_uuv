#! /usr/bin/env python
import roslib
roslib.load_manifest('vanttec_uuv')
import rospy
import actionlib
import time
from geometry_msgs.msg import Pose
from vanttec_uuv.msg import rotateAction
from nav_v1 import uuv_instance
import math

class rotateServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('rotate', rotateAction, self.execute, False)
        self.server.start()
        self.isrot = False
        self.ismov = False
        self.ned_x = None
        self.ned_y = None
        self.ned_z = None
        self.yaw = None
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        self.uuv = uuv_instance()
        time.sleep(1)
        self.waypointtarget = 0        

        self.rot_control = -1 
        self.uuv = uuv_instance()
        rospy.logwarn(self.uuv.waypoints.heading_setpoint )
        rospy.loginfo("rotate server loaded OK")


    def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
        uuv = self.uuv
        if self.rot_control == -1:
            self.uuv.waypoints.heading_setpoint = uuv.yaw
            self.rot_control = 0

        print(goal)
        r = rospy.Rate(25)
        self.waypointstatus = uuv.waypoints.heading_setpoint
        rospy.loginfo(self.waypointstatus)

        #self.waypointtarget = self.waypointstatus + goal.goal_angle
        #rospy.logwarn("Waypoint target is")
        #rospy.logwarn(self.waypointtarget)
        #rospy.logwarn(uuv.yaw)
        #rospy.logwarn(self.waypointstatus)

        self.waypointtarget = self.waypointstatus + goal.goal_angle
        #rospy.logwarn("changing")
        #rospy.logwarn(self.waypointstatus)

        #rospy.logwarn(self.waypointtarget)

        # if self.waypointstatus > math.pi:
        #     rospy.logwarn("MORE")
        #     self.waypointstatus = self.waypointstatus - math.pi
        # elif self.waypointtarget < -math.pi:
        #     rospy.logwarn("Minor")
        #     self.waypointtarget = self.waypointtarget + math.pi
        #     rospy.logwarn ("NEW is")
        #     rospy.logwarn(self.waypointtarget)
        #     self.waypointtarget = math.pi - self.waypointtarget
        #     rospy.logwarn(self.waypointtarget)


        # rospy.logwarn(self.waypointstatus)
        # rospy.logwarn(self.waypointtarget)
        # rospy.logwarn("TARGET OK")
        # if self.waypointtarget >= math.pi:
        #     self.waypointtarget = self.waypointtarget - math.pi
        # elif self.waypointtarget <= -math.pi:
        #     self.waypointtarget =self.waypointtarget + math.pi


        while self.rot_control == 0:
            self.waypointstatus = uuv.waypoints.heading_setpoint
            # rospy.logwarn("starting W")
            # rospy.logwarn(self.waypointtarget)
            if (uuv.waypoints.heading_setpoint < self.waypointtarget) and self.rot_control == 0:
                uuv.waypoints.guidance_law = 0
                uuv.waypoints.heading_setpoint += math.pi/400
                uuv.waypoints.waypoint_list_x = [0 ,0]
                uuv.waypoints.waypoint_list_y = [0, 0]
                uuv.waypoints.waypoint_list_z = [0,0]
                uuv.desired(uuv.waypoints)
                rospy.loginfo("sweeping")
                # rospy.logwarn(uuv.waypoints.heading_setpoint)
                #rospy.loginfo(self.waypointstatus)
                r.sleep()
                # rospy.logwarn("Primero")

                #rospy.logwarn(self.waypointtarget + self.waypointstatus)

                if abs(self.waypointtarget + self.waypointstatus ) < 0.1:
                    self.rot_control = 1


            elif (uuv.waypoints.heading_setpoint > self.waypointtarget) and self.rot_control == 0:
                uuv.waypoints.guidance_law = 0
                uuv.waypoints.heading_setpoint -= math.pi/400
                uuv.waypoints.waypoint_list_x = [0 ,0]
                uuv.waypoints.waypoint_list_y = [0, 0]
                uuv.waypoints.waypoint_list_z = [0,0]
                uuv.desired(uuv.waypoints)
                rospy.loginfo("sweeping")
                # rospy.logwarn(uuv.waypoints.heading_setpoint)
                # rospy.loginfo(self.waypointstatus)
                r.sleep()
                # rospy.logwarn("Segundo")

                #rospy.logwarn(self.waypointtarget - self.waypointstatus)
                if abs(self.waypointtarget - self.waypointstatus) < 0.1:
                    self.rot_control = 1
                    #rospy.logwarn("Terminado Seg")
                    #rospy.logwarn(self.rot_control)
                    #rospy.logwarn(abs(self.waypointtarget - self.waypointstatus))

            

            
            if self.rot_control == 1:
                self._result = 1
                rospy.logwarn('Succeeded rotating')
                #rospy.logwarn(uuv.waypoints.heading_setpoint)
                #rospy.loginfo(self.waypointstatus)
                self.rot_control = 2
                r.sleep()
                break
        
        if self.rot_control == 2:
            rospy.loginfo('Succeeded rotating')
            self.uuv.waypoints.heading_setpoint = uuv.yaw

            self.server.set_succeeded()
            self.rot_control = 0
            
    
    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z  
        


if __name__ == '__main__':
    rospy.init_node('rotate_server')
    server = rotateServer()
    rospy.spin()