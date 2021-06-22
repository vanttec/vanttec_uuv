#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped
from vanttec_uuv.msg import GuidanceWaypoints
from usv_perception.msg import obj_detected, obj_detected_list
from nav_msgs.msg import Path
from vanttec_uuv.scripts.binmission import BinMission
from vanttec_uuv.scripts.gatemission import GateMission
from vanttec_uuv.scripts.buoymission import BuoyMission
# Class Definition
class StateMachine:
    def __init__(self):
        self.transition_mission = 0
        self.activated = True
        self.binmission = BinMission()
        self.gatemission = GateMission()
        self.buoymission = BuoyMission()
        
      
    def transition(self):
        if self.transition_mission == 0 :
            self.gatemission.activate()
            self.transition_mission = 1
        elif self.transition_mission == 1:
            self.buoymission.activate()
            self.transition_mission = 2
        elif self.transition_mission == 2:
            self.binmission.activate()
            self.transition_mission = 4
        
        
def main():
    rospy.init_node("state_machine", anonymous=False)
    rate = rospy.Rate(20)
    state_machine = StateMachine()
    while not rospy.is_shutdown() and state_machine.activated:
        rospy.loginfo(state_machine.transition_mission)
        if(state_machine.transition_mission != 4):
            rospy.loginfo("StateMachine is activated")
            state_machine.transition()
        else:
            rospy.loginfo("StateMachine has finished")
            state_machine.activated = False
        rate.sleep()
    rospy.spin()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
