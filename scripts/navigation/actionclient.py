#! /usr/bin/env python

import roslib
roslib.load_manifest('vanttec_uuv')
import rospy
import actionlib
import time
import math


from vanttec_uuv.msg import rotateAction, rotateGoal, walkAction, walkGoal, gotoAction, gotoGoal
from octomap_server.msg import oclustAction, oclustGoal

if __name__ == '__main__':
    rospy.init_node('action_client')
    # rclient = actionlib.SimpleActionClient('rotate', rotateAction)
    # rclient.wait_for_server()
    # wclient = actionlib.SimpleActionClient('walk', walkAction)
    # wclient.wait_for_server()
    # oclient = actionlib.SimpleActionClient('goto', gotoAction)
    # oclient.wait_for_server()
    oclient = actionlib.SimpleActionClient('oclust', oclustAction)
    oclient.wait_for_server()

    
    ogoal = oclustGoal()
    oclient.send_goal(ogoal)
    oclient.wait_for_result()
    rospy.logwarn("Acabe")
    
    # ggoal = gotoGoal()
    # rospy.loginfo(ggoal)
    # ggoal.goto_point.x = 10
    # ggoal.goto_point.y = 0
    # ggoal.goto_point.z = 0
    # # rgoal.goal_angle = 0
    # print(ggoal)
    # oclient.send_goal(ggoal)
    # oclient.wait_for_result()
    # time.sleep(2)

    # rgoal = rotateGoal()
    # rgoal.goal_angle = math.pi/2
    # # rgoal.goal_angle = 0
    # print(rgoal)
    # rclient.send_goal(rgoal)
    # rclient.wait_for_result()
    # time.sleep(2)

    # wgoal = walkGoal()
    # wgoal.walk_dis = 3
    # print(wgoal)
    # # Fill in the goal here
    # wclient.send_goal(wgoal)
    # wclient.wait_for_result()
