#! /usr/bin/env python

import roslib
roslib.load_manifest('vanttec_uuv')
import rospy
import actionlib
import time
import math


from vanttec_uuv.msg import rotateAction, rotateGoal, walkAction, walkGoal

if __name__ == '__main__':
    rospy.init_node('action_client')
    rclient = actionlib.SimpleActionClient('rotate', rotateAction)
    rclient.wait_for_server()
    wclient = actionlib.SimpleActionClient('walk', walkAction)
    wclient.wait_for_server()

    rgoal = rotateGoal()
    rgoal.goal_angle = math.pi/2
    # rgoal.goal_angle = 0
    print(rgoal)
    rclient.send_goal(rgoal)
    rclient.wait_for_result()
    time.sleep(2)

    wgoal = walkGoal()
    wgoal.walk_dis = 3
    print(wgoal)
    # Fill in the goal here
    wclient.send_goal(wgoal)
    wclient.wait_for_result()
