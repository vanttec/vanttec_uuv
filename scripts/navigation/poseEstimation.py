#!/usr/bin/env python
# -- coding: utf-8 --

from cgitb import enable
from distutils.ccompiler import new_compiler
import math
from pickle import FALSE
from re import X
import rospy
from geometry_msgs.msg import Pose, Point
from vanttec_uuv.msg import obj_detected_list, poses, poseList
import actionlib
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
import math

#Posicion actual
currentPose = Point()

#Modelo de la camara frontal
camera = PinholeCameraModel()
msg = CameraInfo()
msg.D = [-0.813149231, 612.338476, -0.00268726812, -0.00601155767, 1.48938401]
msg.K = [5816.60691, 0.0, 328.23376, 0.0, 5263.47548, 217.7909, 0.0, 0.0, 1.0]
msg.R = [0.999864, 0.008003, -0.014412, -0.007971, 0.999966, 0.00223, 0.01443, -0.002115, 0.999894]
msg.P = [554.3826904296875, 0.0, 320.0, 0.0, 0.0, 554.3826904296875, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
camera.fromCameraInfo(msg) 

#Callback que nos da la posicion actual del uuv
def ins_pose_callback(pose):
    global currentPose
    currentPose.x = pose.position.x
    currentPose.y = pose.position.y
    currentPose.z = pose.position.z

def pixelto3D(u, v, d):
    #Obtener el rayo 3D de la camara al pixel indicado
    ray = camera.projectPixelTo3dRay((u,v))

    #Multiplicar por la distancia para obtener la coordenada 
    #(se cambia el orden de las x, y y z debido a que hay una rotacion en el frame de referencia de la imagen)
    (y, z, x) = [el * (d+400)/1000 for el in ray]
    
    return (x, y, z)


def detected_objects_callback(msg):
    global currentPose
    objects = poseList()
    pub = rospy.Publisher("/uuv_perception/objects", poseList, queue_size=10)
    #Regresar lista con coordenadas de cada uno
    for object in msg.objects:
        item = poses()

        x, y, z = pixelto3D(object.X, object.Y, object.Depth)
        item.location.x = x + currentPose.x
        item.location.y = y + currentPose.y
        item.location.z = z + currentPose.z
        item.name = object.clase

        objects.targets.append(item)
        objects.len += 1

    pub.publish(objects)
    
        
#Current pose
rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, ins_pose_callback)
        
#Object detector
rospy.Subscriber('/uuv_perception/yolo_zed/objects_detected', obj_detected_list, detected_objects_callback,queue_size=10)


if __name__ == "__main__":
    try:
        rospy.init_node("pose_estimation", anonymous=False)
        rospy.spin()

    except rospy.ROSInterruptException:     
        pass