#! /usr/bin/env python

import roslib
roslib.load_manifest('vanttec_uuv')
import rospy
import actionlib

import math


from vanttec_uuv.msg import rotateAction, rotateGoal

if __name__ == '__main__':
    rospy.init_node('rotate_client')
    client = actionlib.SimpleActionClient('rotate', rotateAction)
    client.wait_for_server()

    goal = rotateGoal()
    goal.goal_angle = math.pi/2
    print(goal)
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result()