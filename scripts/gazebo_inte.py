#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

def rot_mat_321(yaw,pitch,roll):
    #3-2-1 Euler angles rotation matrix
    J = np.matrix([[np.cos(pitch)*np.cos(yaw),     np.cos(pitch)*np.sin(yaw),      -np.sin(pitch)],
                    [(np.sin(roll)*np.sin(pitch)*np.cos(yaw))-(np.cos(roll)*np.sin(yaw)),   (np.sin(roll)*np.sin(pitch)*np.sin(yaw))+(np.cos(roll)*np.cos(yaw)),    np.sin(roll)*np.cos(pitch)],
                    [(np.cos(roll)*np.sin(pitch)*np.cos(yaw))+(np.sin(roll)*np.sin(yaw)),   (np.cos(roll)*np.sin(pitch)*np.sin(yaw))-(np.sin(roll)*np.cos(yaw)),    np.cos(roll)*np.cos(pitch)]])
    return J

def DCM_add(yaw1,pitch1,roll1,yaw2,pitch2,roll2):
    RN = rot_mat_321(yaw1,pitch1,roll1)
    BR = rot_mat_321(yaw2,pitch2,roll2)
    BN = np.matmul(BR,RN)
    return BN

def euler_angles_from_DCM(yaw1,pitch1,roll1,yaw2,pitch2,roll2):
    mat = DCM_add(yaw1,pitch1,roll1,yaw2,pitch2,roll2)

    yaw = np.arctan2(mat[0,1],mat[0,0])
    pitch = -np.arcsin(mat[0,2])
    roll = np.arctan2(mat[1,2],mat[2,2])

    return yaw,pitch,roll

def state_callback(msg,pub_state):
    model_msg = ModelState()
    model_msg.model_name = "vtec_u3"

    quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
    # rospy.loginfo(quat)

    # Body to NED
    yaw_b_ned, pitch_b_ned, roll_b_ned = euler_from_quaternion(quat,axes='szyx') #3-2-1 Euler angles
    # rospy.loginfo("Body to NED")
    # rospy.loginfo(yaw_b_ned)
    # rospy.loginfo(pitch_b_ned)
    # rospy.loginfo(roll_b_ned)

    # NED to ENU
    # YPR: -90,0,180 deg
    yaw_ned_enu, pitch_ned_enu, roll_ned_enu= [-np.pi/2, 0, np.pi]
    # rospy.loginfo("NED to ENU")
    # rospy.loginfo(yaw_ned_enu)
    # rospy.loginfo(pitch_ned_enu)
    # rospy.loginfo(roll_ned_enu)

    # Body to ENU
    yaw_b_enu, pitch_b_enu, roll_b_enu = euler_angles_from_DCM(yaw_ned_enu, pitch_ned_enu, roll_ned_enu, yaw_b_ned, pitch_b_ned, roll_b_ned)
    # rospy.loginfo("Body to ENU")
    # rospy.loginfo(yaw_b_enu)
    # rospy.loginfo(pitch_b_enu)
    # rospy.loginfo(roll_b_enu)

    # q_rot = quaternion_from_euler(np.pi, 0, -np.pi/2,axes='szyx')
    # q = quaternion_multiply([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w],q_rot)

    q = quaternion_from_euler(yaw_b_enu, pitch_b_enu, roll_b_enu,axes='szyx')

    rospy.loginfo(q)

    model_msg.pose.position.x = msg.position.y
    model_msg.pose.position.y = msg.position.x
    model_msg.pose.position.z = -msg.position.z - 2

    model_msg.pose.orientation.x = q[0]#0
    model_msg.pose.orientation.y = q[1]#0
    model_msg.pose.orientation.z = q[2]#0.7068252#0.3826834
    model_msg.pose.orientation.w = q[3]#0.7073883#0.9238796

    model_msg.reference_frame = "world"

    pub_state.publish(model_msg)


def main():
    rospy.init_node("gazebo_interface", anonymous=False)
    set_state = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, state_callback, set_state)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
