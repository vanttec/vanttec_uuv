#!/usr/bin/env python3
# -- coding: utf-8 --
"""
import math
import time
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped,Point
from vanttec_uuv.msg import GuidanceWaypoints, obj_detected_list, clusters_center_list
from nav_msgs.msg import Path
import threading
import numpy as np
import pandas as pd 
from sklearn.cluster import KMeans
from joblib import Parallel, delayed
from octomap_server.msg import points_list
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion

class Bottle:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name
# Class Definition
class GrabBottle:
    def __init__(self):
        # ROS Subscribers
        rospy.Subscriber("grab_bottle",String,self.bottle_callback)
        self.grab_bottle=""
        self.can_poses_gazebo = []
        self.currentAttach = None
        self._blockListDict = {
            'can_1': Bottle('bottle', 'bottle_base')
        }
        rospy.loginfo("Waiting for /gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.pose_array_p = rospy.Publisher("pose_array/cans", PoseArray, queue_size=5)
        rospy.loginfo("Waiting for /link_attacher_node/attach Service")
        rospy.wait_for_service("/link_attacher_node/attach")
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        rospy.loginfo("Waiting for /link_attacher_node/detach Service")
        rospy.wait_for_service("/link_attacher_node/detach")
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        loadCansThread = threading.Thread(target=self.load_bottle_gazebo, args=(True, ))
        loadCansThread.start()
    def load_bottle_gazebo(self, wait = False):
        rospy.logwarn("Loading Gazebo Requirements For Attacher...")
        pa = PoseArray()
        pa.header.frame_id = "world_ned"
        pa.header.stamp = rospy.Time.now()
    
        for block in self._blockListDict.itervalues():
            blockName = str(block._name)
            resp_coordinates = self.get_model_srv(blockName, "world_ned")
            pa.poses.append(resp_coordinates.pose)
            self.can_poses_gazebo.append([resp_coordinates.pose.position.x, resp_coordinates.pose.position.y, resp_coordinates.pose.position.z, block])
            rospy.logwarn(blockName)
        self.pose_array_p.publish(pa)
        self.generate_attach_arm_to_can_msg(self._blockListDict)

    def bottle_callback(self,msg):
        self.grab_bottle=msg.data

    def generate_attach_arm_to_can_msg(self, can_name = None):
        if can_name is None or len(self.can_poses_gazebo) == 0:
            return
        req = AttachRequest()
        req.model_name_1 = "vtec_u3" # Robot
        req.link_name_1 = "vtec_u3_base_link"
        req.model_name_2 = str(self.can_poses_gazebo[0][3]._name) # Model can
        req.link_name_2 = str(self.can_poses_gazebo[0][3]._relative_entity_name)  # Link can

        self.currentAttach = req
    def attach_can(self):
        if self.currentAttach == None:
            return
        self.attach_srv.call(self.currentAttach)
        rospy.logwarn("Attached" + str(self.currentAttach.model_name_1) + " with " + str(self.currentAttach.model_name_2))
    def detach_can(self):
        if self.currentAttach == None:
            return
        self.detach_srv.call(self.currentAttach)
        rospy.logwarn("Detached" + str(self.currentAttach.model_name_1) + " with " + str(self.currentAttach.model_name_2))
        self.currentAttach = None
        self.load_bottle_gazebo()
    def main(self):
        if self.grab_bottle=="Attach":
            self.attach_can()
        elif self.grab_bottle=="Detach":
            self.detach_can()

def main():
    rospy.init_node("grab_bottle", anonymous=False)
    rate = rospy.Rate(20)
    mission = GrabBottle()
    while not rospy.is_shutdown():
        mission.main()
        rate.sleep()
    rospy.spin()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:     
        pass
"""