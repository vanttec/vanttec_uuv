#!/usr/bin/env python
# -- coding: utf-8 --

import math
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped, Point
from vanttec_uuv.msg import GuidanceWaypoints, obj_detected_list, obj_detected, Gate
from nav_msgs.msg import Path

# Class Definition
class Gate_Mission:
    def __init__(self):
        self.ned_x = 0
        self.ned_y = 0
        self.ned_z = 0
        self.yaw = 0
        self.activated = True
        self.state = -1
        self.search_state = -1
        self.distance = 0
        self.waypoints = GuidanceWaypoints()
        self.uuv_path = Path()

        self.search_x = 0.0
        self.search_y = 0.0
        self.search_z = 0.0
        self.sweep_state = -1             # 1: start
        self.found_state = -2

        self.enter_waypoint = 0.0
        self.leave_waypoint = 0.0

        self.time_wait=0.0

        self.camera_offset_x = 0.8
        self.camera_offset_y = 0.0
        self.camera_offset_z = -0.1

        self.object = obj_detected()
        self.objects_list = obj_detected_list()

        self.gate_found = 0
        self.party = 'gangster'
        self.y_label = 0.0

        self.gate_angle = 0
        self.gate_angle_body = 0
        self.final_gate_position = Point()
        self.right_gate_position = Point()
        self.left_gate_position = Point()
        self.mid_gate_position = Point()
        self.police_x_coord = 0.0
        self.gangster_x_coord = 0.0

        # ROS Subscribers
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        rospy.Subscriber('/uuv_perception/yolo_zed/objects_detected', obj_detected_list, self.detected_objects_callback)
        # rospy.Subscriber("/uuv_perception/gate_left_waypoint",Point, self.left_gate_callback)
        # rospy.Subscriber("/uuv_perception/gate_rigth_waypoint",Point, self.right_gate_callback)
        rospy.Subscriber("/uuv_perception/gate_position",Gate, self.gate_callback)

        # ROS Publishers
        self.uuv_waypoints = rospy.Publisher("/uuv_guidance/guidance_controller/waypoints", GuidanceWaypoints, queue_size=10)
        self.uuv_path_pub = rospy.Publisher("/uuv_planning/motion_planning/desired_path", Path, queue_size=10)
        self.status_pub = rospy.Publisher("/mission/status", Int32, queue_size=10)
        self.test = rospy.Publisher("/mission/state", Int32, queue_size=10)

    def body_to_ned(self, x, y, z):
        J = np.array([[math.cos(self.yaw),-math.sin(self.yaw),0],
                      [math.sin(self.yaw), math.cos(self.yaw),0],
                      [0,0,1]])
        ned = J.dot(np.array([x,y,z]))
        ned[0] += self.ned_x
        ned[1] += self.ned_y
        ned[2] += self.ned_z
        return ned[0], ned[1], ned[2]

    def gate_to_body(self, x, y, z, side):
        J = np.array([[math.cos(self.gate_angle_body),-math.sin(self.gate_angle_body),0],
                      [math.sin(self.gate_angle_body), math.cos(self.gate_angle_body),0],
                      [0,0,1]])
        body = J.dot(np.array([x,y,z]))
        body[0] += side.x
        body[1] += side.y
        body[2] += side.z
        return body[0], body[1], body[2]

    def ned_to_gate(self, x, y, z):
        J = np.array([[math.cos(self.gate_angle),-math.sin(self.gate_angle),0],
                      [math.sin(self.gate_angle), math.cos(self.gate_angle),0],
                      [0,0,1]])
        J = np.linalg.inv(J)
        n = np.array([x - self.mid_gate_position.x, y - self.mid_gate_position.y, z - self.mid_gate_position.z])
        gate = J.dot(n)
        return gate[0], gate[1], gate[2]

    def gate_to_ned(self, x, y, z, side):
        (x_body,y_body,z_body) = self.gate_to_body(x,y,z,side)
        (ned_x,ned_y,ned_z) = self.body_to_ned(x_body,y_body,z_body)
        return ned_x,ned_y,ned_z

    def detected_objects_callback(self, msg):
        self.objects_list = msg
        for object in msg.objects:
            # if   object.clase == "gate":
            #     self.gate_found = 1
            if   object.clase == "gangster":
                self.gangster_x_coord = object.x
            elif object.clase == "police":
                self.police_x_coord = object.x

        #Waypoint test instead of perception node
        #This array shall be modified with zed inputs of distance

    def gate_callback(self, msg):
        self.gate_found = 1

        # Body
        mid_gate_position_x = msg.gate_position.z + self.camera_offset_x
        mid_gate_position_y = msg.gate_position.x + self.camera_offset_y
        mid_gate_position_z = msg.gate_position.y + self.camera_offset_z

                # Estan invertidos
        right_gate_position_x = msg.left_side.z + self.camera_offset_x
        right_gate_position_y = msg.left_side.x + self.camera_offset_y
        right_gate_position_z = msg.left_side.y + self.camera_offset_z
                # Estan invertidos
        left_gate_position_x = msg.right_side.z + self.camera_offset_x
        left_gate_position_y = msg.right_side.x + self.camera_offset_y
        left_gate_position_z = msg.right_side.y + self.camera_offset_z

        self.gate_angle_body = math.atan2(mid_gate_position_y,mid_gate_position_x)

        (x, y, z) = self.body_to_ned(mid_gate_position_x, mid_gate_position_y, mid_gate_position_z)

        self.mid_gate_position.x = x
        self.mid_gate_position.y = y
        self.mid_gate_position.z = z

        self.gate_angle = math.atan2(y,x)

        (x, y, z) = self.body_to_ned(right_gate_position_x, right_gate_position_y, right_gate_position_z)

        self.right_gate_position.x = x
        self.right_gate_position.y = y
        self.right_gate_position.z = z

        (x, y, z) = self.body_to_ned(left_gate_position_x, left_gate_position_y, left_gate_position_z)

        self.left_gate_position.x = x
        self.left_gate_position.y = y
        self.left_gate_position.z = z

    # def left_gate_callback(self, msg):
    #     # Estan invertidos
    #     self.gate_found = 1
    #     self.right_gate_position.x = msg.x + self.camera_offset_x
    #     self.right_gate_position.y = msg.y + self.camera_offset_y
    #     self.right_gate_position.z = msg.z + self.camera_offset_z

    # def right_gate_callback(self, msg):
    #     # Estan invertidos
    #     self.gate_found = 1
    #     self.left_gate_position.x = msg.x + self.camera_offset_x
    #     self.left_gate_position.y = msg.y + self.camera_offset_y
    #     self.left_gate_position.z = msg.z + self.camera_offset_z

    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z

    def sweep(self,next_mission):
        self.waypoints.guidance_law = 0
        if(self.sweep_state == -1): # Rotate to the left
            if (self.waypoints.heading_setpoint <=  -math.pi/4):
                self.sweep_state = 2
            self.waypoints.heading_setpoint -= math.pi/400.0
            self.waypoints.waypoint_list_x = [0, 0]
            self.waypoints.waypoint_list_y = [0, 0]
            self.waypoints.waypoint_list_z = [0, 0]
            self.desired(self.waypoints)
        elif(self.sweep_state == 2):  # Rotate to the right
            if (self.waypoints.heading_setpoint >= math.pi/4):
                self.sweep_state = 2.1
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint += math.pi/400.0
                self.waypoints.waypoint_list_x = [0, 0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0, 0]
                self.desired(self.waypoints)
        elif(self.sweep_state == 2.1): # Navigate 3 meters front
            if (self.waypoints.heading_setpoint <= 0):
                self.waypoints.guidance_law = 0
                self.search_state = next_mission
                self.search_x = self.ned_x + 3
                self.search_y = self.ned_y
                self.sweep_state = -1
                self.desired(self.waypoints)
            else:
                self.waypoints.guidance_law = 0
                self.waypoints.heading_setpoint -= math.pi/400.0
                self.waypoints.waypoint_list_x = [0, 0]
                self.waypoints.waypoint_list_y = [0, 0]
                self.waypoints.waypoint_list_z = [0, 0]
                self.desired(self.waypoints)

    def wait(self,next_mission):
        rospy.logwarn("Analyzing image with yolo neural network")
        self.waypoints.guidance_law = 0
        self.waypoints.heading_setpoint = 0
        timeduration = rospy.get_time()-self.time_wait
        if self.party == "police":
            if self.police_x_coord < self.gangster_x_coord:
                self.final_gate_position.x = self.left_gate_position.x
                self.final_gate_position.y = self.left_gate_position.y
                self.final_gate_position.z = self.left_gate_position.z
            else:
                self.final_gate_position.x = self.right_gate_position.x
                self.final_gate_position.y = self.right_gate_position.y
                self.final_gate_position.z = self.right_gate_position.z
        else:
            if self.police_x_coord < self.gangster_x_coord:
                self.final_gate_position.x = self.right_gate_position.x
                self.final_gate_position.y = self.right_gate_position.y
                self.final_gate_position.z = self.right_gate_position.z
            else:
                self.final_gate_position.x = self.left_gate_position.x
                self.final_gate_position.y = self.left_gate_position.y
                self.final_gate_position.z = self.left_gate_position.z
        if(timeduration >= 3):
            self.time_wait = 0
            self.found_state = next_mission

    def search(self):
        if(self.gate_found == 0):     # Gate has not been found
            rospy.logwarn("Searching image")
            if self.search_state == -1:
                # self.sweep(0) # Sweep to find gate
                self.search_state = 0
            elif self.search_state == 0:
                self.waypoints.guidance_law = 1
                #move 2 meters
                euc_distance = pow(pow(self.ned_x-self.search_x,2)+pow(self.ned_y-self.search_y,2),0.5)
                if(euc_distance <0.35):
                    self.search_state = -1
                    self.time_wait = rospy.get_time()
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.search_x]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.search_y]
                    self.waypoints.waypoint_list_z = [0,0]
                    self.desired(self.waypoints)
        else:
            if(self.found_state == -2):
                # rospy.logwarn("Analizing the image to choose side")
                self.wait(-1)
            elif(self.found_state == -1):
                rospy.logwarn("Party: " + self.party)
                (x,y,z) = self.gate_to_ned(-1,0,0, self.final_gate_position)
                # self.enter_waypoint = self.ned_x + self.final_gate_position.x - 1
                self.enter_waypoint = x
                (x,y,z) = self.gate_to_ned(0.5,0,0, self.final_gate_position)
                # self.leave_waypoint = self.ned_x + self.final_gate_position.x + 2
                self.leave_waypoint = x
                # self.y_label = self.ned_y + self.final_gate_position.y
                self.y_label = self.final_gate_position.y

                (x,y,z) = self.gate_to_ned(4,0,0, self.mid_gate_position)
                # self.leave_waypoint_x2 = self.ned_x + self.final_gate_position.x + 5
                # self.leave_waypoint_y2 = self.ned_y + self.final_gate_position.y - 1
                self.leave_waypoint_x2 = x
                self.leave_waypoint_y2 = y

                self.found_state = 0
            elif(self.found_state == 0):
                self.waypoints.guidance_law = 1
                rospy.logwarn("Enter gate")
                euc_distance = pow(pow(self.ned_x-self.enter_waypoint,2)+pow(self.ned_y-self.y_label,2),0.5)
                if(euc_distance <0.35):
                    self.found_state = 1
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.enter_waypoint]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.y_label]
                    self.waypoints.waypoint_list_z = [0,0]
                    self.desired(self.waypoints)
            elif(self.found_state == 1):
                self.waypoints.guidance_law = 1
                rospy.logwarn("Exit gate")
                euc_distance = pow(pow(self.ned_x-self.leave_waypoint,2)+pow(self.ned_y-self.y_label,2),0.5)
                if(euc_distance <0.35):
                    self.found_state = 2
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leave_waypoint]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.y_label]
                    self.waypoints.waypoint_list_z = [0,0]
                    self.desired(self.waypoints)
            elif(self.found_state == 2):
                self.waypoints.guidance_law = 1
                rospy.logwarn("Move behind gate")
                euc_distance = pow(pow(self.ned_x-self.leave_waypoint_x2,2)+pow(self.ned_y-self.leave_waypoint_y2,2),0.5)
                if(euc_distance <0.35):
                    self.state = 6
                    self.waypoints.guidance_law = 0
                else:
                    self.waypoints.waypoint_list_x = [self.ned_x,self.leave_waypoint_x2]
                    self.waypoints.waypoint_list_y = [self.ned_y,self.leave_waypoint_y2]
                    self.waypoints.waypoint_list_z = [0,0]
                    self.desired(self.waypoints)

    def gate_mission(self):
        self.waypoints.waypoint_list_length = 2
        self.waypoints.guidance_law = 1

        if(self.state == -1):
            self.search()

    def desired(self, path):
        self.uuv_waypoints.publish(path)
        self.uuv_path.header.stamp = rospy.Time.now()
        self.uuv_path.header.frame_id = "world"
        del self.uuv_path.poses[:]
        for index in range(path.waypoint_list_length):
            pose = PoseStamped()
            pose.header.stamp       = rospy.Time.now()
            pose.header.frame_id    = "world"
            pose.pose.position.x    = path.waypoint_list_x[index]
            pose.pose.position.y    = path.waypoint_list_y[index]
            pose.pose.position.z    = path.waypoint_list_z[index]
            self.uuv_path.poses.append(pose)
        self.uuv_path_pub.publish(self.uuv_path)

    def results(self):
        rospy.loginfo("Gate mission finished")

    def activate(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.activated:
            rospy.loginfo(self.state )
            if(self.state != 6):
                #rospy.loginfo("Gate_mission is activated")
                self.gate_mission()
            else:
                self.activated = False
            rate.sleep()

def main():
    rospy.init_node("gate_mission", anonymous=False)
    rate = rospy.Rate(20)
    gate_mission = Gate_Mission()
    last_detection = []
    rospy.loginfo("Gate_mission is activated")
    while not rospy.is_shutdown() and gate_mission.activated:
        if(gate_mission.state != 6):
            rospy.loginfo("Gate_mission is activated")
            gate_mission.gate_mission()
        else:
            gate_mission.results()
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass