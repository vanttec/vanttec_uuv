#!/usr/bin/env python
# -- coding: utf-8 --

from cgitb import enable
from distutils.ccompiler import new_compiler
import math
from pickle import FALSE
from re import X
import rospy
from std_msgs.msg import Float32MultiArray, Int32, String, Int16
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3, Pose2D
from vanttec_uuv.msg import obj_detected_list, rotateAction, rotateGoal, walkAction, walkGoal, gotoAction, gotoGoal
import actionlib
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
import math

# Default values
RTURN90 = math.pi/2
LTURN90 = -math.pi/2
WALKDIS = 6

class pixel:
    def __init__(self, u = 0, v = 0, d = 0):
        u = u
        v = v
        d = d

class uuv:
    #Inicializacion de uuv
    def __init__(self):

        #Lado
        self.side = "gangster"

        #Action Clients que se encargan del movimiento
        self.rot_client = actionlib.SimpleActionClient('rotate', rotateAction)
        self.rot_goal = rotateGoal()
        self.walk_client = actionlib.SimpleActionClient('walk', walkAction)
        self.walk_goal = walkGoal()
        self.goto_client = actionlib.SimpleActionClient('goto', gotoAction)
        self.goto_goal = gotoGoal()

        rospy.loginfo("Waiting for rotate server")
        self.rot_client.wait_for_server()
        rospy.loginfo("Waiting for walk server")
        self.walk_client.wait_for_server()
        rospy.loginfo("Waiting for goto server")
        self.oclient = actionlib.SimpleActionClient('goto', gotoAction)
        self.oclient.wait_for_server()
        self.ggoal = gotoGoal()
        self.init_pose = Point()
        self.current_pose = Point()
        self.point_handler = Point()

        # ROS Subscribers
        #Current pose
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        #Object detector
        rospy.Subscriber('/uuv_perception/yolo_zed/objects_detected', obj_detected_list, self.detected_objects_callback,queue_size=10)

        #Variables con la posicion inicial
        self.ned = Point()
        self.yaw = 0

        #Pixel y distancia de la imagen del policia
        self.gate_police = pixel()
        self.police_found = 0
        
        #Pixel y distancia de la imagen del mafioso
        self.gate_gangster = pixel()
        self.gangster_found = 0

        #Pixel y distancia de la imagen de la pistola y la placa
        self.buoys_gun = pixel()
        self.buoys_badge = pixel()

        self.gun_found = 0
        self.badge_found = 0

        #Pixel central, pixel inicial, altura, ancho y distancia del gate
        self.gate_u_pixel = 0
        self.gate_v_pixel = 0
        self.gate_x_pixel = 0
        self.gate_y_pixel = 0      
        self.gate_h_pixel = 0
        self.gate_w_pixel = 0
        self.gate_depth = 0

        self.buoys_gun_x_pixel = 0
        self.buoys_gun_y_pixel = 0
        #Altura y anchura del objeto en pixeles
        self.buoys_gun_h_pixel = 0
        self.buoys_gun_w_pixel = 0

        self.buoys_badge_x_pixel = 0
        self.buoys_badge_y_pixel = 0
        #Altura y anchura del objeto en pixeles
        self.buoys_badge_h_pixel = 0
        self.buoys_badge_w_pixel = 0

        #Variables para saber si ya se encontro el gate o el objetivo
        self.gate_found = 0
        self.gate_class_found = 0
        
        self.path = pixel()
        self.path_found = 0

        #Modelo de la camara
        self.camera = PinholeCameraModel()
        msg = CameraInfo()
        msg.D = [-0.813149231, 612.338476, -0.00268726812, -0.00601155767, 1.48938401]
        msg.K = [5816.60691, 0.0, 328.23376, 0.0, 5263.47548, 217.7909, 0.0, 0.0, 1.0]
        msg.R = [0.999864, 0.008003, -0.014412, -0.007971, 0.999966, 0.00223, 0.01443, -0.002115, 0.999894]
        msg.P = [554.3826904296875, 0.0, 320.0, 0.0, 0.0, 554.3826904296875, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera.fromCameraInfo(msg) 

        #Diccionario con las diferentes misiones que se tienen
        self.completedMissions={
            "GATE":0,
            "BUOYS":0,
            "BINS":0,
            "OCTAGON":0
        }

    #Callback que nos da la posicion actual del uuv
    def ins_pose_callback(self, pose):
        self.ned.x = pose.position.x
        self.ned.y = pose.position.y
        self.ned.z = pose.position.z
        self.yaw = pose.orientation.z

    #Main
    def main(self):
        rate = rospy.Rate(10)
        print(rospy.get_time())
        self.init_pose.x = self.ned.x
        self.init_pose.y = self.ned.y

        while not rospy.is_shutdown():
            self.retos()
            rate.sleep()
        rospy.spin()
    
    def rotate(self, rotation = RTURN90):
        self.rot_goal.goal_angle = rotation
        self.rot_client.send_goal(self.rot_goal)
        self.rot_client.wait_for_result()
        rospy.sleep(1)
        

    def walk(self, walk_dis = WALKDIS):
        self.walk_goal.walk_dis = walk_dis
        self.walk_client.send_goal(self.walk_goal)
        self.walk_client.wait_for_result()
        rospy.sleep(1)

    def goto(self, point):
        self.goto_goal.goto_point = point
        self.goto_client.send_goal(self.goto_goal)
        self.goto_client.wait_for_result()


    #Funciones que alinean la camara
    def allign(self):
        while abs(self.yaw) > 0.07:
            # rospy.loginfo(self.uuv.yaw)
            self.rot_goal.goal_angle = RTURN90/90
            self.rot_client.send_goal(self.rot_goal)
            self.rot_client.wait_for_result()

    def lallign(self):
        while abs(self.yaw) > 0.07:
            # rospy.loginfo(self.uuv.yaw)
            self.rot_goal.goal_angle = LTURN90/90
            self.rot_client.send_goal(self.rot_goal)
            self.rot_client.wait_for_result()

    #Callback que nos da los objetos que se han encontrado
    def detected_objects_callback(self, msg):
        self.objects_list = msg
        #Si el UUV detecta a uno o varios objetivos
        for object in msg.objects:
            if object.clase == "badge" and self.completedMissions["BUOYS"] == 0:
                #Pixel central del objeto
                self.buoys_badge.u = object.X
                self.buoys_badge.v = object.Y

                #Pixel inicial del objeto
                self.buoys_badge_x_pixel = object.x
                self.buoys_badge_y_pixel = object.y
                
                #Altura y anchura del objeto en pixeles
                self.buoys_badge_h_pixel = object.h
                self.buoys_badge_w_pixel = object.w
                
                #Distancia del objeto al UUV
                self.buoys_badge.d = object.Depth

                #Marcar objeto como encontrado
                self.badge_found = 1

            elif object.clase == "gun" and self.completedMissions["BUOYS"] == 0:
                #Pixel central del objeto
                self.buoys_gun.u = object.X
                self.buoys_gun.v = object.Y

                #Pixel inicial del objeto
                self.buoys_gun_x_pixel = object.x
                self.buoys_gun_y_pixel = object.y
                
                #Altura y anchura del objeto en pixeles
                self.buoys_gun_h_pixel = object.h
                self.buoys_gun_w_pixel = object.w
                
                #Distancia del objeto al UUV
                self.buoys_gun.d = object.Depth

                #Marcar objeto como encontrado
                self.gun_found = 1
            elif object.clase == "gate" and self.completedMissions["GATE"] == 0:
                #Si detecta al gate, obtener
                #Pixel central del objeto
                self.gate_u_pixel = object.X
                self.gate_v_pixel = object.Y

                #Pixel inicial del objeto
                self.gate_x_pixel = object.x
                self.gate_y_pixel = object.y
                
                #Altura y anchura del objeto en pixeles
                self.gate_h_pixel = object.h
                self.gate_w_pixel = object.w
                
                #Distancia del objeto al UUV
                self.gate_depth = object.Depth

                #Marcar objeto como encontrado
                self.gate_found = 1
            elif object.clase == "gangster" and self.completedMissions["GATE"] == 0:
                #Pixel central del objeto
                self.gate_police.u = object.X
                self.gate_police.v = object.Y
                
                #Distancia del objeto al UUV
                self.gate_police.d = object.Depth

                #Marcar objeto como encontrado
                self.police_found = 1

            elif object.clase == "police" and self.completedMissions["GATE"] == 0:
                #Pixel central del objeto
                self.gate_gangster.u = object.X
                self.gate_gangster.v = object.Y

                #Distancia del objeto al UUV
                self.gate_gangster.d = object.Depth

                #Marcar objeto como encontrado
                self.gangster_found = 1
            elif object.clase == "marker":
                #Pixel central del objeto
                self.path.u = object.X
                self.path.v = object.Y

                #Distancia del objeto al UUV
                self.path.d = object.Depth

                self.path_found = 1
    
    def retos(self):
        #Si encontramos el gate y la imagen correcta
        if self.gate_found == 1 and self.completedMissions["GATE"] == 0:
            #Pasar por el lado del policia
            if self.side == "police" and self.police_found:
                self.gate(self.gate_police.u, self.gate_police.v, self.gate_police.d)
            #Pasar por el lado del gangster
            elif self.gangster_found:
                self.gate(self.gate_gangster.u, self.gate_gangster.v, self.gate_gangster.d)

        elif self.gun_found and self.side == "gangster" and self.completedMissions["BUOYS"] == 0:
            self.buoys(self.buoys_badge.u, self.buoys_badge.v, self.buoys_badge.d)
        elif self.badge_found and self.side == "police" and self.completedMissions["BUOYS"] == 0:
            self.buoys(self.buoys_gun.u, self.buoys_gun.v, self.buoys_gun.d)
        elif self.path_found:
            self.marker(self.path.u, self.path.v, self.path.d)
        
    def scan(self):
        #Check left side
        self.rotate(LTURN90/2)
        self.rotate(RTURN90/2)
        print("scanning")
        #Check right side
        self.rotate(RTURN90/2)
        self.rotate(LTURN90/2)

        

    def pixelto3D(self, u, v, d):
        #Obtener el rayo 3D de la camara al pixel indicado
        ray = self.camera.projectPixelTo3dRay((u,v))

        #Multiplicar por la distancia para obtener la coordenada
        (y, z, x) = [el * (d+400)/1000 for el in ray]
        
        return (x, y, z)

    def gate(self, u, v, d):
        #Obtener coordenada en x, y, z del objetivo
        (x, y, z) = self.pixelto3D(u, v, d)

        distance = (d+400)/1000 
        print(distance)

        if distance > 5:
            x = x * (0.5)
            y = y * (0.5)

            #Convertir pasar a world frame
            x = x + self.ned.x
            y = y + self.ned.y
            z = self.ned.z

            target = Point(x,y,z)
            self.goto(target)
        else:
            #Convertir pasar a world frame
            x = x + self.ned.x
            y = y + self.ned.y
            z = self.ned.z

            print("Object world location: X: " + str(x) + " Y: " + str(y) + " Z: " + str(z))
        
            target = Point(x,y,z)
            self.goto(target)

            #Atravesar el gate
            self.walk(2)
            
            #Marcar el reto como completado
            self.completedMissions["GATE"] = 1
            rospy.loginfo("Done")
    

    def buoys(self, u, v, d):
        #Obtener coordenada en x, y, z del objetivo
        (x, y, z) = self.pixelto3D(u, v, d)

        distance = (d+400)/1000 
        print(distance)

        if distance > 5:
            x = x * (0.5)
            y = y * (0.5)

            #Convertir pasar a world frame
            x = x + self.ned.x
            y = y + self.ned.y
            z = self.ned.z

            print("Object world location: X: " + str(x) + " Y: " + str(y) + " Z: " + str(z))
        
            target = Point(x,y,z)
            self.goto(target)
        else:
            #Convertir pasar a world frame
            
            x = x + self.ned.x
            y = y + self.ned.y
            z = self.ned.z

            print("Object world location: X: " + str(x) + " Y: " + str(y) + " Z: " + str(z))

            #Tocar el objetivo
            target = Point(x,y,z)
            self.goto(target)
            rospy.loginfo("BOOP")
            self.rotate(LTURN90)
            self.walk(2)
            self.rotate(RTURN90)

            #Wait for 5 seconds
            rospy.sleep(5)
            
            #Marcar el reto como completado
            self.completedMissions["BUOYS"] = 1
            rospy.loginfo("Done")

    def octagon(self, u, v, d):
        #Obtener coordenada en x, y, z del objetivo
        (x, y, z) = self.pixelto3D(u, v, d)

        #TODO
        #do stuff
    
    def marker(self, u, v, d):
        #Obtener coordenada en x, y, z del objetivo
        (x, y, z) = self.pixelto3D(u, v, d)
        print("markers")
        x += self.ned.x
        y += self.ned.y
        
        #If marker seen go to marker
        target = Point(x,y,self.ned.z)
        self.goto(target)
        
        self.scan()
        


if __name__ == "__main__":
    try:
        rospy.init_node("navigation", anonymous=False)
        i = uuv()
        i.main()
    except rospy.ROSInterruptException:     
        pass