#include <ros/ros.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vanttec_uuv/Obstacle.h>
#include <vanttec_uuv/detectedObstacles.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose2D.h>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define _USE_MATH_DEFINES

using namespace Eigen;

/*
struct obj_list {
    enum {is_float, is_char} type;
    union  
    {
        char  objeto;
        float x;
        float y;
        float z;
        float orientation;
        float radio;

    }val;
};*/
 union obj_list  
    {
        char  objeto;
        float x;
        float y;
        float z;
        float orientation;
        float radio;

    };
//const float SAMPLE_TIME_S       = 0.01;
//const float DEFAULT_SPEED_MPS   = 0.2;

class ObstacleSimulator{

    public:
    obj_list lista_objetos[2];
    float ned_x = 0;
    float ned_y = 0;
    float yaw = 0;
    float challenge = 0;
    float max_visible_radius = 10;
    ros::NodeHandle nh_;
    ros::Publisher marker_pub; //= n.advertise<visualization_msgs::Marker>("Obstacle_markers",1000);
    ros::Publisher detector_pub;
    ros::Subscriber pose_sub =nh_.subscribe("uuv_pose",1000,&ObstacleSimulator::poseCallback,this);
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
    ObstacleSimulator(int challenge_select,ros::NodeHandle nh)
    {
        nh_ = nh;
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("Obstacle_markers",1000);
        detector_pub = nh_.advertise<vanttec_uuv::detectedObstacles>("detected_objects",1000);
        challenge = challenge_select;
        if (challenge == 0){

            //Gate
            
            lista_objetos[0].objeto = 'gate';
            lista_objetos[0].x = 1; 
            lista_objetos[0].y = 0; 
            lista_objetos[0].z = -1; 
            lista_objetos[0].orientation = 0;
            lista_objetos[0].radio = 1;

            //Marker
 
            lista_objetos[1].objeto = 'marker';
            lista_objetos[1].x = 13; 
            lista_objetos[1].y = 0; 
            lista_objetos[1].z = -1; 
            lista_objetos[1].orientation = 0;
            lista_objetos[1].radio = 1;
        }
        else if (challenge == 2){

        }
    }
    void poseCallback(const geometry_msgs::Pose2D msg)
    {
        this->ned_x = msg.x;
        this->ned_y = msg.y;
    }
    void simulate()
    {   
        vanttec_uuv::detectedObstacles obstaculos_detectados;
        int k = 0;
        for (int i = 0; i< sizeof(this->lista_objetos)/sizeof(obj_list);i++)
        {
            float x = this->lista_objetos[i].x;
            float y = this->lista_objetos[i].y;
            float z = this->lista_objetos[i].z;
            float orientation = this->lista_objetos[i].orientation;
            float radio = this->lista_objetos[i].radio;
                   
           float delta_x = x-this->ned_x;
           float delta_y = y-this->ned_y;
           float distance = pow(delta_x*delta_x+delta_y*delta_y,0.5);
           if (distance < this->max_visible_radius)
           {
               ROS_INFO("UN OBJETO EN RADIO DE DISTANCIA");
               this->ned_to_body(&x,&y);
               if(x>1)
               {
                   vanttec_uuv::Obstacle obstaculo_act;
                   obstaculo_act.pose.position.x = x;
                   obstaculo_act.pose.position.y = y;
                   obstaculo_act.pose.position.z = z;
                   obstaculo_act.pose.orientation.x = 0;
                   obstaculo_act.pose.orientation.y = 0;
                   obstaculo_act.pose.orientation.z = orientation;
                   obstaculo_act.pose.orientation.w = 0;
                   obstaculo_act.tipo = this->lista_objetos[i].objeto;
                   obstaculo_act.radio = radio;
                   k ++;
                   obstaculos_detectados.obstacles.resize(k);
                   obstaculos_detectados.obstacles[k-1] = obstaculo_act;
                  
               }
           } 
        }
        this->detector_pub.publish(obstaculos_detectados);
        //publicas el arreglo de obstaculos    
        
    }
    void ned_to_body( float *x2, float *y2 )
    {
        Vector2d u(*x2,*y2);
        
        Matrix2d J = this->rotation_matrix(this->yaw);
        MatrixXd rot = J*u;
        *x2 = rot.coeff(0,0)+this->ned_x;
        *y2 = rot.coeff(1,0)+this->ned_y;
        

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
            if (this->challenge == 0){

                                                
                
                marker.header.frame_id  = "/world";
                marker.lifetime = ros::Duration(0.1);

                //Primer poste gate
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = 1;
                marker.pose.position.y = -1;
                marker.pose.position.z = -1.5;
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
                
                marker.id = 0;

                marker_array.markers.push_back(marker);
               
                //Segundo poste gate 

                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = 1;
                marker.pose.position.y = 1;
                marker.pose.position.z = -1.5;
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
                marker.id = 1;
                
                marker_array.markers.push_back(marker);

                //Poste transversal
                tf2::Quaternion quat;
                geometry_msgs::Quaternion quat_msg;
                quat.setRPY(M_PI_2,0,0);
                quat_msg = tf2::toMsg(quat);
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = 1;
                marker.pose.position.y = 0;
                marker.pose.position.z = -1;
                marker.pose.orientation = quat_msg;
                marker.scale.x = 0.0762;
                marker.scale.y = 0.0762;
                marker.scale.z = 2;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.id = 2;
                marker_array.markers.push_back(marker);
                
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = 13;
                marker.pose.position.y = 0;
                marker.pose.position.z = -1;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.0762;
                marker.scale.y = 0.0762;
                marker.scale.z = 2;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.id = 3;
                marker_array.markers.push_back(marker);

                this->marker_pub.publish(marker_array);
                //this->marker_pub.publish(marker2);

            }
        

    }


};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_obstacle_simulation_node"); 
    ros::NodeHandle n;
    ObstacleSimulator obstacleSim(0,n);
    ros::Rate loop_rate(10);
    
    while(ros::ok()){
        obstacleSim.rviz_markers();
        obstacleSim.simulate();
        ros::spinOnce();
        loop_rate.sleep();

    } 
    return 0;
}