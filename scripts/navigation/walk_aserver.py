#! /usr/bin/env python3
# import roslib
# roslib.load_manifest('vanttec_uuv')
import rospy
import actionlib

from geometry_msgs.msg import Pose
from vanttec_uuv.msg import walkAction
from nav_v1 import uuv_instance
import math

class walkServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('walk', walkAction, self.execute, False)
    self.server.start()
    self.isrot = False
    self.ismov = False
    self.waypointstatus = None
    self.waypointtarget = None
    self.walk_control = 0 
    self.uuv = uuv_instance()
    self.th = None
    self.ned_x = None
    self.ned_y = None
    self.ned_z = None
    self.yaw = None
    rospy.loginfo("walk server loaded OK")
    rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)


  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    uuv = self.uuv
    print(goal)
    r = rospy.Rate(10)
    uuv.waypoints.guidance_law = 1
    #move 3 meter
    rospy.loginfo("Moving")
    rospy.logwarn(goal.walk_dis)
    print(self.yaw)
    uuv.searchx = uuv.ned_x + ((math.cos(self.yaw)) * goal.walk_dis)
    uuv.searchy = uuv.ned_y + ((math.sin(self.yaw)) * goal.walk_dis)
    #rospy.loginfo(uuv.searchx - uuv.ned_x)
    #rospy.loginfo(uuv.searchy - uuv.ned_y)
    
    _euc_distance = pow(pow(uuv.ned_x-uuv.searchx,2)+pow(uuv.ned_y-uuv.searchy,2),0.5)
    # rospy.loginfo(_euc_distance)
    self.th = 0.35
    while _euc_distance >= self.th:
      _euc_distance = pow(pow(uuv.ned_x-uuv.searchx,2)+pow(uuv.ned_y-uuv.searchy,2),0.5)
      # rospy.loginfo(_euc_distance)
      uuv.waypoints.waypoint_list_x = [uuv.ned_x,uuv.searchx]
      uuv.waypoints.waypoint_list_y = [uuv.ned_y,uuv.searchy]
      uuv.waypoints.waypoint_list_z = [0,0]   
      uuv.desired(uuv.waypoints)
      r.sleep()
      if _euc_distance < 0.35:
        self.searchstate = -1
        self.walk_control  = 1
        break
    if self.walk_control == 1 :
      rospy.loginfo('Succeeded Walking')
      self.server.set_succeeded()

  def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z







if __name__ == '__main__':
  rospy.init_node('walk_server')
  server = walkServer()
  rospy.spin()