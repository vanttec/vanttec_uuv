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


using namespace Eigen;

//const float SAMPLE_TIME_S       = 0.01;
//const float DEFAULT_SPEED_MPS   = 0.2;

class ObstacleSimulator{

    public:
    obj lista_objetos;
    float ned_x = 0;
    float ned_y = 0;
    float yaw = 0;
    float challenge = 0;
    float max_visible_radius = 10;
    ros::NodeHandle nh_;
    ros::Publisher marker_pub; //= n.advertise<visualization_msgs::Marker>("Obstacle_markers",1000);
    ros::Publisher detector_pub;
    ros::Subscriber pose_sub nh_.subscribe("uuv_pose",1000,this->poseCallback);
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    
    ObstacleSimulator(int challenge_select)
    {
        marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("Obstacle_markers",1000);
        detector_pub nh_.advertise<vanttec_uuv::detectedObstacles>("detected_objects",1000);
        challenge = challenge_select;
        if (challenge == 0){
            obj lista_objetos[2];

            //Gate
            lista_objetos[0].type = is_char;
            lista_objetos[0].val.objeto = 'gate';
            lista_objetos[0].type = is_float;
            lista_objetos[0].val.x = 0; 
            lista_objetos[0].val.y = 3; 
            lista_objetos[0].val.z = -1; 
            lista_objetos[0].val.orientation = 0;
            lista_objetos[0].val.radio = 1;

            //Marker
            lista_objetos[1].type = is_char;
            lista_objetos[1].val.objeto = 'marker';
            lista_objetos[1].type = is_float;
            lista_objetos[1].val.x = 0; 
            lista_objetos[1].val.y = 13; 
            lista_objetos[1].val.z = -1; 
            lista_objetos[1].val.orientation = 0;
            lista_objetos[1].val.radio = 1;
        }
        else if (challenge == 2){

        }
    }
    void poseCallback(const geomtry_msgs::Pose2D msg)
    {
        this->ned_x = msg->x;
        this->ned_y = msg->y;
    }
    void simulate()
    {   
        vanttec_uuv::detectedObstacles obstaculos_detectados;
        int k = 0;
        for (int i = 0; i< sizeof(this->lista_objetos)/sizeof(obj);i++)
        {
            switch (this->lista_objetos[i].type)
            {
                case is_float:
                    float x = this->lista_objetos[i].x;
                    float y = this->lista_objetos[i].y;
                    float z = this->lista_objetos[i].z;
                    float orientation = this->lista_objetos[i].orientation;
                    float radio = this->lista_objetos[i].radio;
                    break;
                case is_char:
                    char tipo_obstaculo = this->lista_objetos[i].objeto
            }
           float delta_x = x-this->ned_x;
           float delta_y = y-this->ned_y;
           float distance = pow(delta_x*delta_x+delta_y*delta_y,0.5);
           if (distance < this->max_visible_radius)
           {
               this->ned_to_body(&x,&y);
               if(x>1)
               {
                   vanttec_uuv::Obstacle obstaculo_act;
                   obstaculo_act.pose.position.x = x;
                   obstaculo_act.pose.position.y = y;
                   obstaculo_act.pose.position.z = z;
                   obstaculo_act.orientation.x = 0
                   obstaculo_act.orientation.y = 0
                   obstaculo_act.orientation.z = orientation;
                   obstaculo_act.orientation.w = 0
                   obstaculo_act.class = tipo_obstaculo;
                   obstaculo_act.radio = radio;
                   k ++;
                   obstaculos_detectados.obstacles.resize(k);
                   obstaculos_detectados.obstacles[k-1] = obstaculo_act;
                  
               }
           } 
        }
        detector_pub.publish(obstaculos_detectados);
        //publicas el arreglo de obstaculos    
        
    }
    void ned_to_body( float *x2, float *y2 )
    {
        MatrixXf a(2,1);
        a << *x2,*y2;
        Matrix2d J = this->rotation_matrix(this->yaw);
        Matrix2d rot = J.dot(a);
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

                marker_array.markers.resize(2);                                  
                
                marker.header.frame_id  = "/world";
                marker.header.stamp = ros::Time::now();

                //Primer poste gate
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = 1;
                marker.pose.position.y = -0.5;
                marker.pose.position.z = -1.5;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.762;
                marker.scale.y = 0.762;
                marker.scale.z = 1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                marker_array.markers[0] =  marker;

                //Segundo poste gate 

                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = 1;
                marker.pose.position.y = 0.5;
                marker.pose.position.z = -1.5;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.762;
                marker.scale.y = 0.762;
                marker.scale.z = 1;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                this->marker_pub.publish(marker1);
                //this->marker_pub.publish(marker2);

            }
        

    }


};


struct obj {
    enum {is_int, is_float, is_char} type;
    union  
    {
        char  objeto;
        float x;
        float y;
        float z;
        float orientation;
        float radio;

    }val ;
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_obstacle_simulation_node"); 
    ros::Rate loop_rate(10);
    ObstacleSimulator obstacleSim(0);
    while(ros::ok()){
        obstacleSim.rviz_markers();
        ros::spinOnce();
        loop_rate.sleep();

    } 
    return 0;
}