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
from image_geometry import PinholeCameraModel, StereoCameraModel
from sensor_msgs.msg import CameraInfo
import math

#Posicion actual
# currentPose = Point()

# currentPose.x = 0
# currentPose.y = 0
# currentPose.z = 0

#Modelo de la camara
camera = PinholeCameraModel()
#Modelo de la camara stereo
stereo = StereoCameraModel()


def infoCamRightCB(msg):
    stereo.right.fromCameraInfo(msg)

def infoCamLeftCB(msg):
    stereo.left.fromCameraInfo(msg)

def infoCamCB(msg):
    camera.fromCameraInfo(msg) 

def stereoPixelto3D(u,v,d):
    #left cam u, v
    disparity = stereo.getDisparity(d) #ver disparity map
    x, y, z = stereo.projectPixelTo3d((u,v), disparity)
    return y, z, x

# #Callback que nos da la posicion actual del uuv
# def ins_pose_callback(pose):
#     global currentPose
#     currentPose.x = pose.position.x
#     currentPose.y = pose.position.y
#     currentPose.z = pose.position.z

def pixelto3D(u, v, d):
    #Obtener el rayo 3D de la camara al pixel indicado
    ray = camera.projectPixelTo3dRay((u,v))

    #Multiplicar por la distancia para obtener la coordenada 
    #(se cambia el orden de las x, y y z debido a que hay una rotacion en el frame de referencia de la imagen)
    (y, z, x) = [el * (d) for el in ray]
    
    return (x, y, z)


def detected_objects_callback(msg):
    global currentPose
    objects = poseList()
    pub = rospy.Publisher("/uuv_perception/objects", poseList, queue_size=10)
    #Regresar lista con coordenadas de cada uno
    for object in msg.objects:
        item = poses()

        x, y, z = pixelto3D(object.X, object.Y, object.Depth)
        rospy.loginfo("Pose Monocular X: " + str(x) + " Y:" + str(y) + " Z:" + str(z))

        x, y, z = stereoPixelto3D(object.X, object.Y, object.Depth)
        rospy.loginfo("Pose Stereo X: " + str(x) + " Y:" + str(y) + " Z:" + str(z))

        item.location.x = x 
        item.location.y = y 
        item.location.z = z 
        item.name = object.clase

        objects.targets.append(item)
        objects.len += 1

    pub.publish(objects)
    
        
#Current pose
# rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, ins_pose_callback)
        
#Object detector
rospy.Subscriber('/uuv_perception/yolo_zed/objects_detected', obj_detected_list, detected_objects_callback, queue_size=10)
rospy.Subscriber('/zed2i/zed_node/right_raw/camera_info', CameraInfo, infoCamRightCB, queue_size=1)
rospy.Subscriber('/zed2i/zed_node/left_raw/camera_info', CameraInfo, infoCamLeftCB, queue_size=1)


if __name__ == "__main__":
    try:
        rospy.init_node("pose_estimation", anonymous=False)
        rospy.spin()

    except rospy.ROSInterruptException:     
        pass