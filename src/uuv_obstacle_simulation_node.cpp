#include <ros/ros.h>
#include <stdio.h>
#include <Eigen/Dense>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>


using namespace Eigen;

//const float SAMPLE_TIME_S       = 0.01;
//const float DEFAULT_SPEED_MPS   = 0.2;

class ObstacleSimulator{

    public:

    float ned_x = 0;
    float ned_y = 0;
    float yaw = 0;
    float challenge = 0;
    ros::Publisher marker_pub; //= n.advertise<visualization_msgs::Marker>("Obstacle_markers",1000);
    
    void ned_to_body(float ned_x2, float ned_y2, float* x2, float* y2 )
    {

    }

    Matrix2d rotation_matrix(float angle)
    {
        Matrix2d a;
        //a.resize(2,2);
        a << cos(angle), -1*sin(angle),
             sin(angle), cos(angle);
        return a;
    }

    void rviz_markers(ros::NodeHandle *nh)
    {
            this->marker_pub = nh->advertise<visualization_msgs::Marker>("Obstacle_markers",1000);
            visualization_msgs::Marker marker1,marker2,marker3;
            visualization_msgs::MarkerArray marker_array;
            if (this->challenge == 0){
                //Primer poste gate
                marker1.header.frame_id  = "/world";
                marker1.header.stamp = ros::Time();
                marker1.type = visualization_msgs::Marker::CYLINDER;
                marker1.action = visualization_msgs::Marker::ADD;
                marker1.pose.position.x = 1;
                marker1.pose.position.y = -0.5;
                marker1.pose.position.z = -1.5;
                marker1.pose.orientation.x = 0.0;
                marker1.pose.orientation.y = 0.0;
                marker1.pose.orientation.z = 0.0;
                marker1.pose.orientation.w = 1.0;
                marker1.scale.x = 0.762;
                marker1.scale.y = 0.762;
                marker1.scale.z = 1;
                marker1.color.a = 1.0;
                marker1.color.r = 0.0;
                marker1.color.g = 1.0;
                marker1.color.b = 0.0;

                //Segundo poste gate 
                marker2.header.frame_id  = "/world";
                marker2.header.stamp = ros::Time();
                marker2.type = visualization_msgs::Marker::CYLINDER;
                marker2.action = visualization_msgs::Marker::ADD;
                marker2.pose.position.x = 1;
                marker2.pose.position.y = 0.5;
                marker2.pose.position.z = -1.5;
                marker2.pose.orientation.x = 0.0;
                marker2.pose.orientation.y = 0.0;
                marker2.pose.orientation.z = 0.0;
                marker2.pose.orientation.w = 1.0;
                marker2.scale.x = 0.762;
                marker2.scale.y = 0.762;
                marker2.scale.z = 1;
                marker2.color.a = 1.0;
                marker2.color.r = 0.0;
                marker2.color.g = 1.0;
                marker2.color.b = 0.0;

                this->marker_pub.publish(marker1);
                //this->marker_pub.publish(marker2);

            }
        

    }


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_obstacle_simulation_node");
    ros::NodeHandle nh;    
    ros::Rate loop_rate(10);
    ObstacleSimulator obstacleSim;
    while(ros::ok()){
        obstacleSim.rviz_markers(&nh);
        ros::spinOnce();
        loop_rate.sleep();

    } 
    return 0;
}