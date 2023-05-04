#! /usr/bin/env python

import roslib
roslib.load_manifest('vanttec_uuv')
import rospy
import actionlib
import time
import math
import numpy as np


from vanttec_uuv.msg import rotateAction, rotateGoal, walkAction, walkGoal, gotoAction, gotoGoal
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

        
class client:
    def __init__(self):
        rospy.loginfo("Waiting for action servers")    
        self.rot_client = actionlib.SimpleActionClient('rotate', rotateAction)
        self.rot_goal = rotateGoal()
        self.walk_client = actionlib.SimpleActionClient('walk', walkAction)
        self.walk_goal = walkGoal()
        self.goto_client = actionlib.SimpleActionClient('goto', gotoAction)
        self.goto_goal = gotoGoal()
        rospy.loginfo("Navigation Servers loaded ")

        n = 50
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Plot a helix along the x-axis
        theta_max = 4 * np.pi
        theta = np.linspace(0, theta_max, n)
        #Point calculations
        self.x = theta
        self.z = np.sin(theta)
        self.y = np.cos(theta)
        ax.plot(self.x, self.y, self.z, 'b', lw=2)
        # An line through the centre of the helix
        ax.plot((-theta_max*0.2, theta_max * 1.2), (0,0), (0,0), color='k', lw=2)
        ax.plot(self.x, self.y, 0, color='r', lw=1, alpha=0.5)
        ax.plot(self.x, [0]*n, self.z, color='m', lw=1, alpha=0.5)
        # Remove axis planes, ticks and labels
        ax.set_axis_off()
        plt.show()


    def helicoidal(self):
        for i in range(len(self.x)):
            self.goto_goal.goto_point.x = self.x[i]
            self.goto_goal.goto_point.y = self.y[i]
            self.goto_goal.goto_point.z = self.z[i]
            self.goto_client.send_goal(self.goto_goal)
            self.goto_client.wait_for_result()
    
    def main(self):
        self.helicoidal()
    


if __name__ == "__main__":
    try:
        rospy.init_node("ActionClient", anonymous=False)
        i = client()
        i.main()
    except rospy.ROSInterruptException:     
        pass