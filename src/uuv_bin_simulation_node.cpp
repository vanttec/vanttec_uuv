#include <vanttec_uuv/Obstacle.h>
#include <vanttec_uuv/DetectedObstacles.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>
#include <Eigen/Dense>

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#define _USE_MATH_DEFINES

using namespace Eigen;

const float SAMPLE_TIME_S       = 0.01;
const float DEFAULT_SPEED_MPS   = 0.2;

typedef struct obj_list
{
    char  objeto;
    float x;
    float y;
    float z;
    float orientation;
    float radio;  
} obj_list;

class ObstacleSimulator
{
    public: 
        
        obj_list lista_objetos[2];
        
        float ned_x;
        float ned_y;
        float yaw;
        int challenge;
        float max_visible_radius;
        float fov_angle;
        float fov_slope;
        float fov_x_offset;

        ros::NodeHandle     nh_;
        ros::Publisher      marker_pub;
        ros::Publisher      fov_pub;
        ros::Publisher      detector_pub;
        ros::Subscriber     pose_sub;
    
    ObstacleSimulator(int challenge_select, ros::NodeHandle nh)
    {
        this->ned_x = 0;
        this->ned_y = 0;
        this->yaw = 0;
        this->challenge = 0;
        this->max_visible_radius = 3.5;
        this->fov_angle = M_PI / 6.0;
        this->fov_x_offset = 0.225;

        this->fov_slope = ((sin(this->fov_angle) - this->fov_x_offset)/cos(this->fov_angle));
        
        this->nh_ = nh;
        this->marker_pub        = this->nh_.advertise<visualization_msgs::MarkerArray>("/object_simulator/obstacle_markers", 1000);
        this->fov_pub           = this->nh_.advertise<visualization_msgs::MarkerArray>("/object_simulator/fov", 1000);

        this->detector_pub      = this->nh_.advertise<vanttec_uuv::DetectedObstacles>("/object_simulator/detected_objects", 1000);
        this->pose_sub          = this->nh_.subscribe("/uuv_simulation/dynamic_model/pose",
                                                        1000,
                                                        &ObstacleSimulator::poseCallback,
                                                        this);

        this->challenge = challenge_select;

        switch(this->challenge)
        {
            case 0:
                // Square
                this->lista_objetos[0].objeto         = 's1';
                this->lista_objetos[0].x              = 3.0; 
                this->lista_objetos[0].y              = -1.0; 
                this->lista_objetos[0].z              = -2.2; 
                this->lista_objetos[0].orientation    = 0.0;
                this->lista_objetos[0].radio          = 1;
                /*// Square
                this->lista_objetos[1].objeto         = 's2';
                this->lista_objetos[1].x              = 3.0; 
                this->lista_objetos[1].y              = -1.0; 
                this->lista_objetos[1].z              = -2.2; 
                this->lista_objetos[1].orientation    = 0.0;
                this->lista_objetos[1].radio          = 1;
                // Square
                this->lista_objetos[1].objeto         = 's3';
                this->lista_objetos[1].x              = 3.0; 
                this->lista_objetos[1].y              = -1.0; 
                this->lista_objetos[1].z              = -2.2; 
                this->lista_objetos[1].orientation    = 0.0;
                this->lista_objetos[1].radio          = 1;
                // Square
                this->lista_objetos[1].objeto         = 's4';
                this->lista_objetos[1].x              = 3.0; 
                this->lista_objetos[1].y              = -1.0; 
                this->lista_objetos[1].z              = -2.2; 
                this->lista_objetos[1].orientation    = 0.0;
                this->lista_objetos[1].radio          = 1;*/
                //Marker
                /*
                lista_objetos[1].objeto = 'marker';
                lista_objetos[1].x = 13; 
                lista_objetos[1].y = 0; 
                lista_objetos[1].z = -1; 
                lista_objetos[1].orientation = 0;
                lista_objetos[1].radio = 1;
                */
                break;
            default:
                break;

        }
    }

    void poseCallback(const geometry_msgs::Pose& msg)
    {
        this->ned_x     = msg.position.x;
        this->ned_y     = msg.position.y;
        this->yaw       = -msg.orientation.z;
    }

    void simulate()
    {   
        vanttec_uuv::DetectedObstacles obstaculos_detectados;
        int k = 0;
        for (int i = 0; i< sizeof(this->lista_objetos)/sizeof(obj_list);i++)
        {
            float x = this->lista_objetos[i].x - this->ned_x;
            float y = this->lista_objetos[i].y - this->ned_y;
            float z = this->lista_objetos[i].z;
            float orientation = this->lista_objetos[i].orientation;
            float radio = this->lista_objetos[i].radio;

            std::cout << "NED G " << this->lista_objetos[i].x << ", " << this->lista_objetos[i].y << std::endl;
            std::cout << "NED S " << this->ned_x << ", " << this->ned_y << ", " << this->yaw << std::endl;

            this->ned_to_body(&x, &y);

            
            float expected_x;

            if (y >= 0)
            {
                expected_x = (1.8 * this->fov_slope * y) + this->fov_x_offset;
            }
            else 
            {
                expected_x = -(1.8 * this->fov_slope * y) + this->fov_x_offset;
            }
            
            std::cout << expected_x << std::endl;
            std::cout << x << ", " << y << std::endl;

            
            float cone_end = 0;

            if (abs(y) <= this->max_visible_radius)
            {
                cone_end = sqrt(pow(this->max_visible_radius, 2) - pow(y,2));
            }
            
            std::cout << cone_end << std::endl; 

            if((x >= expected_x) && (x <= cone_end))
            {
                vanttec_uuv::Obstacle obstaculo_act;
                obstaculo_act.pose.position.x = x;
                obstaculo_act.pose.position.y = y;
                obstaculo_act.pose.position.z = z;
                obstaculo_act.pose.orientation.x = 0;
                obstaculo_act.pose.orientation.y = 0;
                obstaculo_act.pose.orientation.z = orientation;
                obstaculo_act.pose.orientation.w = 0;
                obstaculo_act.type = this->lista_objetos[i].objeto;
                obstaculo_act.radius = radio;
                k++;
                //obstaculos_detectados.obstacles.resize(k);
                obstaculos_detectados.obstacles.push_back(obstaculo_act); 
                ROS_INFO("UN OBJETO EN RADIO DE DISTANCIA");
            }   
        }  
    
        this->detector_pub.publish(obstaculos_detectados);       
    }

    void ned_to_body(float *x2, float *y2)
    {
        Vector2d u(*x2,*y2);
        
        Matrix2d J = this->rotation_matrix(this->yaw);
        MatrixXd rot = J*u;

        *x2 = rot.coeff(0,0);
        *y2 = rot.coeff(1,0);
    }

    Matrix2d rotation_matrix(float angle)
    {
        Matrix2d a;
        //a.resize(2,2);
        a << cos(angle), -1*sin(angle),
             sin(angle), cos(angle);
        return a;
    }

    void rviz_markers()
    {
        //this->marker_pub = nh->advertise<visualization_msgs::Marker>("Obstacle_markers",1000);
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker marker_fov;
        visualization_msgs::MarkerArray marker_array_fov;

        // FOV Cone
        marker_fov.header.frame_id  = "/uuv";
        marker_fov.lifetime = ros::Duration(0.1);

        tf2::Quaternion quat;
        geometry_msgs::Quaternion quat_msg;
        quat.setRPY(M_PI_2, 0, this->fov_angle);
        quat_msg = tf2::toMsg(quat);
        marker_fov.type = visualization_msgs::Marker::CYLINDER;
        marker_fov.action = visualization_msgs::Marker::ADD;
        marker_fov.pose.position.x = (0.225/2) + this->max_visible_radius/2 * sin(this->fov_angle);
        marker_fov.pose.position.y = -this->max_visible_radius/2 * cos(this->fov_angle);
        marker_fov.pose.position.z = 0.190;
        marker_fov.pose.orientation = quat_msg;
        marker_fov.scale.x = 0.00762;
        marker_fov.scale.y = 0.00762;
        marker_fov.scale.z = this->max_visible_radius - this->fov_x_offset;
        marker_fov.color.a = 1.0;
        marker_fov.color.r = 0.0;
        marker_fov.color.g = 1.0;
        marker_fov.color.b = 0.5;
        marker_fov.id = 1;

        marker_array_fov.markers.push_back(marker_fov);

        quat.setRPY(M_PI_2, 0, -this->fov_angle);
        quat_msg = tf2::toMsg(quat);
        marker_fov.type = visualization_msgs::Marker::CYLINDER;
        marker_fov.action = visualization_msgs::Marker::ADD;
        marker_fov.pose.position.x = (0.225/2) + this->max_visible_radius/2 * sin(this->fov_angle);
        marker_fov.pose.position.y = this->max_visible_radius/2 * cos(this->fov_angle);
        marker_fov.pose.position.z = 0.190;
        marker_fov.pose.orientation = quat_msg;
        marker_fov.scale.x = 0.00762;
        marker_fov.scale.y = 0.00762;
        marker_fov.scale.z = this->max_visible_radius - this->fov_x_offset;
        marker_fov.color.a = 1.0;
        marker_fov.color.r = 0.0;
        marker_fov.color.g = 1.0;
        marker_fov.color.b = 0.5;
        marker_fov.id = 2;

        marker_array_fov.markers.push_back(marker_fov);

        quat.setRPY(M_PI_2, 0, M_PI_2);
        quat_msg = tf2::toMsg(quat);
        marker_fov.type = visualization_msgs::Marker::CYLINDER;
        marker_fov.action = visualization_msgs::Marker::ADD;
        marker_fov.pose.position.x = this->max_visible_radius/2 + (0.225/2);
        marker_fov.pose.position.y = 0;
        marker_fov.pose.position.z = 0.190;
        marker_fov.pose.orientation = quat_msg;
        marker_fov.scale.x = 0.00762;
        marker_fov.scale.y = 0.00762;
        marker_fov.scale.z = this->max_visible_radius - this->fov_x_offset;
        marker_fov.color.a = 1.0;
        marker_fov.color.r = 0.0;
        marker_fov.color.g = 1.0;
        marker_fov.color.b = 0.5;
        marker_fov.id = 3;

        marker_array_fov.markers.push_back(marker_fov);

        this->fov_pub.publish(marker_array_fov);
        
        if (this->challenge == 0)
        {                                          
            marker.header.frame_id  = "/world";
            marker.lifetime = ros::Duration(0.1);
            
            //Poste Transversal Vertical
            tf2::Quaternion quat1;
            geometry_msgs::Quaternion quat_msg1;
            quat1.setRPY(0, M_PI_2, M_PI_2);
            quat_msg1 = tf2::toMsg(quat1);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x;
            marker.pose.position.y = -this->lista_objetos[0].y;
            marker.pose.position.z = -this->lista_objetos[0].z;
            marker.pose.orientation = quat_msg1;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 1;
            marker_array.markers.push_back(marker);
            
            //Poste Transversal Horizontal
            tf2::Quaternion quat2;
            geometry_msgs::Quaternion quat_msg2;
            quat2.setRPY(M_PI_2, M_PI_2, M_PI_2);
            quat_msg2 = tf2::toMsg(quat2);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x;
            marker.pose.position.y = -this->lista_objetos[0].y;
            marker.pose.position.z = -this->lista_objetos[0].z;
            marker.pose.orientation = quat_msg2;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 2;
            marker_array.markers.push_back(marker);

            //Poste Este
            tf2::Quaternion quatEste;
            geometry_msgs::Quaternion quat_msgEste;
            quatEste.setRPY(0, M_PI_2, M_PI_2);
            quat_msgEste = tf2::toMsg(quatEste);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x - this->lista_objetos[0].radio;
            marker.pose.position.y = -this->lista_objetos[0].y;
            marker.pose.position.z = -this->lista_objetos[0].z;
            marker.pose.orientation = quat_msgEste;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 3;
            marker_array.markers.push_back(marker);

            //Poste Oeste
            tf2::Quaternion quatOeste;
            geometry_msgs::Quaternion quat_msgOeste;
            quatOeste.setRPY(0, M_PI_2, M_PI_2);
            quat_msgOeste = tf2::toMsg(quatOeste);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x + this->lista_objetos[0].radio;
            marker.pose.position.y = -this->lista_objetos[0].y;
            marker.pose.position.z = -this->lista_objetos[0].z;
            marker.pose.orientation = quat_msgOeste;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 4;
            marker_array.markers.push_back(marker);

            //Poste Norte
            tf2::Quaternion quatNorte;
            geometry_msgs::Quaternion quat_msgNorte;
            quatNorte.setRPY(M_PI_2, M_PI_2, M_PI_2);
            quat_msgNorte = tf2::toMsg(quatNorte);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x;
            marker.pose.position.y = -this->lista_objetos[0].y + this->lista_objetos[0].radio;
            marker.pose.position.z = -this->lista_objetos[0].z;
            marker.pose.orientation = quat_msgNorte;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 5;
            marker_array.markers.push_back(marker);

            //Poste Sur
            tf2::Quaternion quatSur;
            geometry_msgs::Quaternion quat_msgSur;
            quatSur.setRPY(M_PI_2, M_PI_2, M_PI_2);
            quat_msgSur = tf2::toMsg(quatSur);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x;
            marker.pose.position.y = -this->lista_objetos[0].y - this->lista_objetos[0].radio;
            marker.pose.position.z = -this->lista_objetos[0].z;
            marker.pose.orientation = quat_msgSur;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 6;
            marker_array.markers.push_back(marker);

            //Primer objeto
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x + this->lista_objetos[0].radio/2;
            marker.pose.position.y = -this->lista_objetos[0].y + this->lista_objetos[0].radio/2;
            marker.pose.position.z = -this->lista_objetos[0].z - 0.2;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.254;
            marker.scale.y = 0.254;
            marker.scale.z = 0.254;
            marker.color.a = 1;
            marker.color.r = 0.027;
            marker.color.g = 0.447;
            marker.color.b = 0.710;
            marker.id = 7;
            marker_array.markers.push_back(marker); 
            this->marker_pub.publish(marker_array); 

            //Segundo objeto
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x - this->lista_objetos[0].radio/2;
            marker.pose.position.y = -this->lista_objetos[0].y + this->lista_objetos[0].radio/2;
            marker.pose.position.z = -this->lista_objetos[0].z - 0.2;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.254;
            marker.scale.y = 0.254;
            marker.scale.z = 0.254;
            marker.color.a = 1;
            marker.color.r = 0.027;
            marker.color.g = 0.447;
            marker.color.b = 0.710;
            marker.id = 8;
            marker_array.markers.push_back(marker); 
            this->marker_pub.publish(marker_array); 

            //Tercer objeto
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x + this->lista_objetos[0].radio/2;
            marker.pose.position.y = -this->lista_objetos[0].y - this->lista_objetos[0].radio/2;
            marker.pose.position.z = -this->lista_objetos[0].z - 0.2;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.254;
            marker.scale.y = 0.254;
            marker.scale.z = 0.254;
            marker.color.a = 1;
            marker.color.r = 0.027;
            marker.color.g = 0.447;
            marker.color.b = 0.710;
            marker.id = 9;
            marker_array.markers.push_back(marker); 
            this->marker_pub.publish(marker_array); 

            //Cuarto objeto
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x - this->lista_objetos[0].radio/2;
            marker.pose.position.y = -this->lista_objetos[0].y - this->lista_objetos[0].radio/2;
            marker.pose.position.z = -this->lista_objetos[0].z - 0.2;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.254;
            marker.scale.y = 0.254;
            marker.scale.z = 0.254;
            marker.color.a = 1;
            marker.color.r = 0.027;
            marker.color.g = 0.447;
            marker.color.b = 0.710;
            marker.id = 10;
            marker_array.markers.push_back(marker); 
            this->marker_pub.publish(marker_array);          
          /*  //Primer poste bin
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x;
            marker.pose.position.y = -this->lista_objetos[0].y;
            marker.pose.position.z = -this->lista_objetos[0].z + 0.5;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 1.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            
            marker.id = 0;

            marker_array.markers.push_back(marker);
            
            //Segundo poste bin 

            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x - cos(this->lista_objetos[0].orientation);
            marker.pose.position.y = -(this->lista_objetos[0].y - sin(this->lista_objetos[0].orientation));
            marker.pose.position.z = -this->lista_objetos[0].z + 0.5;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 2.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 1;
            
            marker_array.markers.push_back(marker);

            //Tercer poste bin
            tf2::Quaternion quat0;
            geometry_msgs::Quaternion quat_msg0;
            quat0.setRPY(M_PI_2, 0, -(this->lista_objetos[0].orientation - M_PI_2));
            quat_msg0 = tf2::toMsg(quat0);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x;
            marker.pose.position.y = -this->lista_objetos[0].y;
            marker.pose.position.z = -this->lista_objetos[0].z - 1;
            marker.pose.orientation = quat_msg0;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            
            marker.id = 2;

            marker_array.markers.push_back(marker);
            
            //Cuarto poste bin 

            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x - cos(this->lista_objetos[0].orientation);
            marker.pose.position.y = -(this->lista_objetos[0].y - sin(this->lista_objetos[0].orientation));
            marker.pose.position.z = -this->lista_objetos[0].z + 0.5;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 3;
            
            marker_array.markers.push_back(marker);
            
            // Poste medio bin 

            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x + cos(this->lista_objetos[0].orientation);
            marker.pose.position.y = -(this->lista_objetos[0].y + sin(this->lista_objetos[0].orientation));
            marker.pose.position.z = -this->lista_objetos[0].z + 0.5;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 4;
            
            marker_array.markers.push_back(marker);

            //Poste transversal
            tf2::Quaternion quat;
            geometry_msgs::Quaternion quat_msg;
            quat.setRPY(M_PI_2, 0, -(this->lista_objetos[0].orientation - M_PI_2));
            quat_msg = tf2::toMsg(quat);
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = this->lista_objetos[0].x;
            marker.pose.position.y = -(this->lista_objetos[0].y);
            marker.pose.position.z = -(this->lista_objetos[0].z - 1);
            marker.pose.orientation = quat_msg;
            marker.scale.x = 0.0762;
            marker.scale.y = 0.0762;
            marker.scale.z = 2;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.id = 5;
            marker_array.markers.push_back(marker);

            this->marker_pub.publish(marker_array);*/
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_obstacle_simulation_node"); 
    ros::NodeHandle     n;
    ObstacleSimulator   obstacleSim(0, n);
    ros::Rate           loop_rate(1.0/SAMPLE_TIME_S);
    
    while(ros::ok())
    {
        ros::spinOnce();

        obstacleSim.rviz_markers();
        obstacleSim.simulate();
    
        loop_rate.sleep();
    } 
    return 0;
}