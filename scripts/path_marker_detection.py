#!/usr/bin/env python
import cv2
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vanttec_uuv.msg import GuidanceWaypoints
from math import atan2, cos, sin, sqrt, pi,radians
import numpy as np

class utils:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False
        self.image = [[]]

        self.ned_x = 0
        self.ned_y = 0
        self.ned_z = 0
        self.yaw = 0
        # Connect image topic
        rospy.Subscriber('/downr200/camera/color/image_raw',Image, self.bottom_camera_img)
        rospy.Subscriber("/uuv_simulation/dynamic_model/pose", Pose, self.ins_pose_callback)
        

        rospy.sleep(1)
        


    def bottom_camera_img(self,data):
        try:
            # To interface ROS and OpenCV by converting ROS images into OpenCV images
            # Convert a sensor_msgs::Image message to an OpenCV cv::Mat.
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
            self.image = cv_img
        except CvBridgeError as e:
            rospy.logerr(e) 
   
    
    def ins_pose_callback(self,pose):
        self.ned_x = pose.position.x
        self.ned_y = pose.position.y
        self.ned_z = pose.position.z
        self.yaw = pose.orientation.z 




# def empty(a):
#     pass


#Posicion del sub
# UUV_position = [2,1,0.5]

#Funcion que construye el stack de imagenes que se despliega al final

# def stackImages(scale,imgArray):
#     rows = len(imgArray)
#     cols = len(imgArray[0])
#     rowsAvailable = isinstance(imgArray[0], list)
#     width = imgArray[0][0].shape[1]
#     height = imgArray[0][0].shape[0]
#     if rowsAvailable:
#         for x in range ( 0, rows):
#             for y in range(0, cols):
#                 if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
#                     imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
#                 else:
#                     imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
#                 if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
#         imageBlank = np.zeros((height, width, 3), np.uint8)
#         hor = [imageBlank]*rows
#         hor_con = [imageBlank]*rows
#         for x in range(0, rows):
#             hor[x] = np.hstack(imgArray[x])
#         ver = np.vstack(hor)
#     else:
#         for x in range(0, rows):
#             if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
#                 imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
#             else:
#                 imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
#             if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
#         hor= np.hstack(imgArray)
#         ver = hor
#     return ver


def getContours_and_markerAngle(img,imgContour,ned_yaw):
    #Utilizacion de findContours para buscar contornos
    _, contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        #Busqueda del area y defenicion de un area minima para descartar ruido u objetos no grandes
        area = cv2.contourArea(contour)
        areaMin =  5000 #cv2.getTrackbarPos("Area", "Parameters") 

        if area > areaMin:
            cv2.drawContours(imgContour, contour, -1, (255, 0, 255), 6)
            perimeter = cv2.arcLength(contour, True)
            borders = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            len_borders = len(borders)
            print(len_borders)
        #Se genera un bounding box que se ajusta lo mas posible a los contornos del objeto, teniendo la misma figura
            if  (len_borders >= 6) and (len_borders <= 9):
                adjustedBox = cv2.minAreaRect(contour)
                boxValues = cv2.boxPoints(adjustedBox)
                boxValues = np.int0(boxValues)

                markerCenter = (int(adjustedBox[0][0]),int(adjustedBox[0][1]))
                markerWidth = int(adjustedBox[1][0])
                markerHeight = int(adjustedBox[1][1])
                angleDegrees = int(adjustedBox[2])
                if markerWidth < markerHeight:
                    pass
                else:
                    angleDegrees = 90 + angleDegrees
                    markerAngle = float(radians(angleDegrees))

            #Bounding box que se desplegara encerrando todo el objeto
                x , y , w, h = cv2.boundingRect(borders)
                cv2.rectangle(imgContour, (x , y ), (x + w , y + h ), (0, 255, 0), 5)
            #Despliegue de valores en la imagen
                cv2.putText(imgContour, "Points: " + str(len(borders)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,
                           (0, 255, 0), 2)
                cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,
                          (0, 255, 0), 2)
                cv2.putText(imgContour,"Angle:" + str(angleDegrees) + "degrees",(x + w + 20, y + 70),cv2.FONT_HERSHEY_COMPLEX, 0.7,  (0, 0, 255), 2)
                print(angleDegrees)
                
            else:
                
                markerAngle = ned_yaw
                print(markerAngle)
            return(markerAngle)

        


def waypoint (markerAngle,waypointMagnitude):

    x = float(waypointMagnitude) * float(cos(markerAngle))
    y = float(waypointMagnitude) * float(sin(markerAngle))
    z = 0
    return (x,y,z)


def rotation_matrix (uuv_orientation, x_waypoint,y_waypoint,z_waypoint,UUV_position_x,UUV_position_y,UUV_position_z):

    rotation = np.array([[cos(uuv_orientation),-sin(uuv_orientation),0],
                        [sin(uuv_orientation), cos(uuv_orientation),0],
                        [0,0,1]])

    transform = rotation.dot(np.array([x_waypoint,y_waypoint,z_waypoint]))
    print(transform)
    ned = transform + np.array([UUV_position_x,UUV_position_y,UUV_position_z])
    print (ned)
    return ned


def main():
    rospy.init_node('marker_detection', anonymous=True)  #Inicializa nodo
         # rospy.Subscriber('/downr200/camera/color/image_raw',Image,bottom_camera_img)
    
    pub = rospy.Publisher("/uuv_guidance/guidance_controller/waypoints", GuidanceWaypoints, queue_size=10)
    # while not rospy.is_shutdown():
    #     ned = ned_waypoint
    #     pub.publish(ned_waypoint)
    rate = rospy.Rate(20)
    rate.sleep()
    #camera =  bottom_camera()
    
    
    
    

    # frameWidth = 640
    # frameHeight = 480
    
    # cap.set(3, frameWidth)
    # cap.set(4, frameHeight)
    
    #cv2.namedWindow("Parameters")
    # cv2.resizeWindow("Parameters",640,240)
    # cv2.createTrackbar("Threshold1","Parameters",25,255,empty)
    # cv2.createTrackbar("Threshold2","Parameters",35,255,empty)
    # cv2.createTrackbar("Area","Parameters",5000,40000,empty)
    
    while True:
    #img=cv2.imread("Marker_dataset/pathmarker_98.png") #7 puntos, (imagen normal)
    #img=cv2.imread("Marker_dataset/rect2.jpg")
    #img=cv2.imread("Marker_dataset/pathmarker_100(2).png") #9 puntos(imagen normal borrosa)
    #img=cv2.imread("Marker_dataset/pathmarker_97(2).png") #Dependiendo del threshold toma 5 u 8 puntos (Mucho salt-pepper imagen normal)
    #img=cv2.imread("Marker_dataset/pathmarker_96(2).png") #Subir ambos thresholds, 9 puntos (salt-pepper en imagen normal)
    #img=cv2.imread("Marker_dataset/pathmarker_78(2).png") #Detecta 8 puntos, el filtro por area se baja (imagen alejada y borosa)
    #img=cv2.imread("Marker_dataset/pathmarker_79(2).png") #Detecta 10 puntos, ambos thresholds deben estar al , el filtro por area lo bajamos (imagen alejada con salt-pepper)
    #img=cv2.imread("Marker_dataset/pathmarker_80.png") #Dectecta 9 puntos, el filtro por area se baja (imagen alejada)

    #Copia de imagen en la que se desplegara el contorno y el bounding box

    #imgContour = camera.image.copy()
        UUV = utils()
        #La imagen de muestra se hace borrosa
        imgBlur = cv2.GaussianBlur(UUV.image, (7, 7), cv2.BORDER_DEFAULT)
        #La  imagen pasa a escala de grises
        imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
        #Definicion de thresholds que necesita el detector de bordes
        threshold1 = 68  #cv2.getTrackbarPos("Threshold1", "Parameters")

        threshold2 = 28 #cv2.getTrackbarPos("Threshold2", "Parameters") 
        #Detector de bordes Canny
        imgCanny = cv2.Canny(imgGray,threshold1,threshold2)
        #Para destacar los bordes de la imagen se define un kernel con una matriz de 5x5 con unos
        kernel = np.ones((5,5))
        #Los bordes de la imagen se dilatan
        imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
        #Se llama a la funcion que obtiene el angulo del marcador en caso de detectar el numero de bordes del marcador
        markerAngle = getContours_and_markerAngle(imgDil,UUV.image,UUV.yaw)
        #Se acomoda el orden de las imagenes que organiza la funcion para el stack
        #imgStack = stackImages(0.8,([UUV.image,imgCanny],
        #                            [imgDil,imgBlur]))


        #Magnitud de la posicion del waypoint
        waypointMagnitude = 1.0
        
        #Suma del heading actual del submarino mas el del marker para obtener el heading deseado

        UUV_heading = markerAngle + UUV.yaw

        x_waypoint,y_waypoint,z_waypoint = waypoint(markerAngle,waypointMagnitude)

        ned_waypoint = rotation_matrix(UUV.yaw,x_waypoint,y_waypoint,z_waypoint,UUV.ned_x,UUV.ned_y,UUV.ned_z)

        Waypoint = GuidanceWaypoints()
        Waypoint.guidance_law = 1
        Waypoint.waypoint_list_length = 2

        Waypoint.waypoint_list_x = [UUV.ned_x,ned_waypoint[0]]
        Waypoint.waypoint_list_y = [UUV.ned_y,ned_waypoint[1]]
        Waypoint.waypoint_list_z = [UUV.ned_z,ned_waypoint[2]]
        Waypoint.heading_setpoint = UUV_heading
            
        pub.publish(Waypoint)

    

        cv2.imshow("Result", UUV.image ) #imgStack

        if cv2.waitKey(1) & 0xFF == ord('q'):
         break
    # cv2.waitKey(0)
    # cv2.destroyAllWindows() #Permite desplegar el stack de imagenes hasta presionar tecla esc
    rospy.spin()


if __name__ == "_main_":
    try:
        main()

    except rospy.ROSInterruptException:
        pass

main()
