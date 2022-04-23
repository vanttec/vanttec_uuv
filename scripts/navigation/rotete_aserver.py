#! /usr/bin/env python
import roslib
roslib.load_manifest('vanttec_uuv')
import rospy
import actionlib

from vanttec_uuv.msg import rotateAction
from nav_v1 import uuv_instance
import math

class rotateServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('rotate', rotateAction, self.execute, False)
    self.server.start()

    self.isrot = False
    self.ismov = False
    self.waypointstatus = None
    self.waypointtarget = None
    self.rot_control = 0 
    self.uuv = uuv_instance()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    uuv = self.uuv
    print(goal)
    r = rospy.Rate(10)
    self.waypointstatus = uuv.waypoints.heading_setpoint
    rospy.loginfo(self.waypointstatus)
    self.waypointtarget = self.waypointstatus + goal.goal_angle
    print(self.waypointtarget)

    while self.rot_control == 0:
        self.waypointstatus = uuv.waypoints.heading_setpoint
        if (uuv.waypoints.heading_setpoint < self.waypointtarget):
                    uuv.waypoints.guidance_law = 0
                    uuv.waypoints.heading_setpoint += math.pi/400
                    uuv.waypoints.waypoint_list_x = [0 ,0]
                    uuv.waypoints.waypoint_list_y = [0, 0]
                    uuv.waypoints.waypoint_list_z = [0,0]
                    uuv.desired(uuv.waypoints)
                    rospy.loginfo("sweeping")
                    rospy.logwarn(uuv.waypoints.heading_setpoint)
                    rospy.loginfo(self.waypointstatus)

        
        else :
            self._result = 1
            rospy.loginfo('Succeeded rotating')
            
            rospy.logwarn(uuv.waypoints.heading_setpoint)
            rospy.loginfo(self.waypointstatus)
            self.rot_control = 1
            r.sleep()

        # else: 
        #     self.rot_control = 1
        #     rospy.logwarn("Sleeping")
        #     rate = rospy.Rate(10)
        #     for i in range(60):
        #         rate.sleep()
        #     self.rot_control = 0
        #     rospy.loginfo(self.waypointstatus)
        #     return 1

    
    if self.rot_control == 1:
        rospy.loginfo('Succeeded rotating')
        self.server.set_succeeded()
        self.rot_control = 0
    
    


if __name__ == '__main__':
  rospy.init_node('rotate_server')
  server = rotateServer()
  rospy.spin()