#!/usr/bin/env python
# -- coding: utf-8 --

import math
import time
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String
from geometry_msgs.msg import Pose, PoseStamped, Point,Twist
from vanttec_uuv.msg import GuidanceWaypoints, obj_detected_list, obj_detected, Gate
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu

# Class Definition
class Kalman_Filter:
    def __init__(self):
        self.ned_x = 0
        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0
        self.ned_y = 0
        self.ned_z = 0
        self.yaw = 0
        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
        self.prevPx = 0
        self.prevPy = 0
        self.prevPz = 0
        self.prevVx = 0
        self.prevVy = 0
        self.prevVz = 0
        # ROS Subscribers
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        rospy.Subscriber("/uuv_simulation/dynamic_model/vel", Twist, self.ins_vel_callback)
        rospy.Subscriber("/uuv_accel", Twist, self.ins_acc_callback)
    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z
    def ins_acc_callback(self,acc):
        self.acc_x = acc.linear.x
        self.acc_y = acc.linear.y
        self.acc_z = acc.linear.z
    def ins_vel_callback(self,vel):
        self.vel_x = vel.linear.x
        self.vel_y = vel.linear.y
        self.vel_z = vel.linear.z

    def main(self):

        dt=1
        C = np.eye(6)
        H = np.eye(6)
        
        

        Ax=self.acc_x
        Ay=self.acc_y
        Az=self.acc_z
        Acceleration=np.array([[Ax],[Ay],[Az]])
        #Measurements
        #Posición y Velocidad en X, leer estimación en t=0;
        Xmeasurements=np.array([self.prevPx,self.ned_x])
        promX=np.sum(Xmeasurements)/len(Xmeasurements) 
        VXmeasurements= np.array([self.prevVx,self.vel_x])
        promVX=np.sum(VXmeasurements)/len(VXmeasurements)

        #Posición y Velocidad en Y, 1er estimación en t=0;
        Ymeasurements = np.array([self.prevPy,self.vel_y])
        promY=np.sum(Ymeasurements)/len(Ymeasurements)
        VYmeasurements = np.array([self.prevVy,self.vel_y])
        promVY=np.sum(VYmeasurements)/len(VYmeasurements)

        #Posición y Velocidad en Z, 1er estimación en t=0;
        Zmeasurements = np.array([self.prevPz,self.ned_z])
        promZ=np.sum(Zmeasurements)/len(Zmeasurements)
        VZmeasurements = np.array([self.prevVz,self.vel_z])
        promVZ=np.sum(VZmeasurements)/len(VZmeasurements)

        # Measurement Erros (p/matriz R)
        # Para hacer estos valores más exactos, se calculan, basados en los datos medidos
        sumvarianzaX = 0
        sumvarianzaVX = 0
        sumvarianzaY = 0
        sumvarianzaVY = 0
        sumvarianzaZ = 0
        sumvarianzaVZ = 0


        for n in range(len(Xmeasurements)):
            sumvarianzaX = sumvarianzaX + (promX-Xmeasurements[n])**2
            sumvarianzaVX = sumvarianzaVX + (promVX-VXmeasurements[n])**2
            sumvarianzaY = sumvarianzaY + (promY-Ymeasurements[n])**2
            sumvarianzaVY = sumvarianzaVY + (promVY-VYmeasurements[n])**2
            sumvarianzaZ = sumvarianzaZ + (promZ-Zmeasurements[n])**2
            sumvarianzaVZ = sumvarianzaVZ + (promVZ-VZmeasurements[n])**2


        # Vamos a trabajar con desviaciones, por los elementos de covarianza
        n=len(Xmeasurements)
        Xerrorm = math.sqrt(sumvarianzaX/n)
        VXerrorm = math.sqrt(sumvarianzaVX/n)

        Yerrorm = math.sqrt(sumvarianzaY/n)
        VYerrorm = math.sqrt(sumvarianzaVY/n)

        Zerrorm = math.sqrt(sumvarianzaZ/n)
        VZerrorm = math.sqrt(sumvarianzaVZ/n)

        #Pprev = [PXerrorm^2, 0; 0, PVerrorm^2];

        

        # Measurements Errors
        R = np.array([[Xerrorm**2, Xerrorm*Yerrorm, Xerrorm*Zerrorm, Xerrorm*VXerrorm, Xerrorm*VYerrorm, Xerrorm*VZerrorm],
                [Yerrorm*Xerrorm, Yerrorm**2, Yerrorm*Zerrorm, Yerrorm*VXerrorm, Yerrorm*VYerrorm, Yerrorm*VZerrorm],
                [Zerrorm*Xerrorm, Zerrorm*Yerrorm, Zerrorm**2, Zerrorm*VXerrorm, Zerrorm*VYerrorm, Zerrorm*VZerrorm],
                [VXerrorm*Xerrorm, VXerrorm*Yerrorm, VXerrorm*Zerrorm, VXerrorm**2, VXerrorm*VYerrorm, VXerrorm*VZerrorm],
                [VYerrorm*Xerrorm, VYerrorm*Yerrorm, VYerrorm*Zerrorm, VYerrorm*VXerrorm, VYerrorm**2, VYerrorm*VZerrorm],
                [VZerrorm*Xerrorm, VZerrorm*Yerrorm, VZerrorm*Zerrorm, VZerrorm*VXerrorm, VZerrorm*VYerrorm, VZerrorm**2]])

        # Covariance Process Errors
        PXerrorm = 0.0001
        PVXerrorm = 0.0001
        PYerrorm = 0.0001
        PVYerrorm = 0.0001
        PZerrorm = 0.0001
        PVZerrorm = 0.0001

        prevP = np.array([[PXerrorm**2, PXerrorm*PYerrorm, PXerrorm*PZerrorm, PXerrorm*PVXerrorm, PXerrorm*PVYerrorm, PXerrorm*PVZerrorm],
            [PYerrorm*PXerrorm, PYerrorm**2, PYerrorm*PZerrorm, PYerrorm*PVXerrorm, PYerrorm*PVYerrorm, PYerrorm*PVZerrorm],
            [PZerrorm*PXerrorm, PZerrorm*PYerrorm, PZerrorm**2, PZerrorm*PVXerrorm, PZerrorm*PVYerrorm, PZerrorm*PVZerrorm],
            [PVXerrorm*PXerrorm, PVXerrorm*PYerrorm, PVXerrorm*PZerrorm, PVXerrorm**2, PVXerrorm*PVYerrorm, PVXerrorm*PVZerrorm],
            [PVYerrorm*PXerrorm, PVYerrorm*PYerrorm, PVYerrorm*PZerrorm, PVYerrorm*PVXerrorm, PVYerrorm**2, PVYerrorm*PVZerrorm],
            [PVZerrorm*PXerrorm, PVZerrorm*PYerrorm, PVZerrorm*PZerrorm, PVZerrorm*PVXerrorm, PVZerrorm*PVYerrorm, PVZerrorm**2]])

        prevP2 = np.array([[PXerrorm**2, 0, 0, 0, 0, 0],
                [0, PYerrorm**2, 0, 0, 0, 0],
                [0, 0, PZerrorm**2, 0, 0, 0],
                [0, 0, 0, PVXerrorm**2, 0, 0],
                [0, 0, 0, 0, PVYerrorm**2, 0],
                [0, 0, 0, 0, 0, PVZerrorm**2]])

        #Definimos matrices bases para definir la posición estimada
        #Matriz A
        A = np.array([[1,0,0,dt,0,0],
            [0,1,0,0,dt,0],
            [0,0,1,0,0,dt],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]])
        # Matriz B define un "factor" que afecta la variable de control (aceleración)
        B = np.array([[(dt**2)/2,0,0],
            [0,(dt**2)/2,0],
            [0,0,(dt**2)/2],
            [dt,0,0],
            [0,dt,0],
            [0,0,dt]])

        prevState = np.array([[Xmeasurements[0]], [Ymeasurements[0]], [Zmeasurements[0]], [VXmeasurements[0]], [VYmeasurements[0]], [VZmeasurements[0]]])
        self.prevPx = self.ned_x
        self.prevPy = self.ned_y
        self.prevPz = self.ned_z
        self.prevVx = self.vel_x
        self.prevVy = self.vel_y
        self.prevVz = self.vel_z

        for t in range(len(Xmeasurements)-1):
            State = np.matmul(A,prevState) + np.matmul(B,Acceleration)
            P = np.matmul(A,(np.matmul(prevP2,(np.transpose(A))))) 
            #Si las variables son independientes, cuidar que solo la diagonal principal se mantenga
            #P = P-[0,P(1,2); P(2,1),0]; #Checar esto porque puede alentar el proceso
            K = np.true_divide((np.matmul(prevP2,np.transpose(H))),(np.matmul(H,np.matmul(prevP2,np.transpose(H))) + R))
            # New measurement for the next loop
            Y = np.matmul(C,np.array([[Xmeasurements[t+1]],[Ymeasurements[t+1]],[Zmeasurements[t+1]],[VXmeasurements[t+1]],[VYmeasurements[t+1]],[VZmeasurements[t+1]]]))
            prevState = State + np.matmul(K,(Y-np.matmul(H,State)))
            #Estimations(t) = prevState(1:3,1)
            prevP2 = np.matmul((H -np.matmul(K,H)),P)
        
        
        rospy.logwarn(State)#,self.ned_y,self.ned_z,self.vel_x,self.vel_y,self.vel_z,self.acc_x,self.acc_y,self.acc_z)


def main():
    rospy.init_node("kalman_filter", anonymous=False)
    rate = rospy.Rate(20)
    kalman_filter = Kalman_Filter()
    rospy.loginfo("Kalman_filter is activated")
    while not rospy.is_shutdown() :
        rospy.loginfo("Kalman_filter is activated")
        kalman_filter.main()
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass