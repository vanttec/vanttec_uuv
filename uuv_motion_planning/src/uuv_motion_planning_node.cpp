#include <uuv_commons.hpp>
#include <uuv_guidance/GuidanceWaypoints.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <ros/ros.h>

const float         SAMPLE_TIME_S           = 0.01;
uint8_t             trajectory_selector     = 0;
uint8_t             path_publish_flag       = 0;
float               circle_radius           = 1.0;

void OnTrajectoryReceive(const std_msgs::UInt8& _trajectory)
{
    trajectory_selector     = (int) _trajectory.data;
    path_publish_flag       = 0;
}

void OnRadiusReceive(const std_msgs::Float32& _radius)
{
    circle_radius = (float) _radius.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_motion_planning");
    ros::NodeHandle nh;
    
    ros::Rate       cycle_rate(int(1 / SAMPLE_TIME_S));
    
    ros::Publisher  uuv_waypoints = nh.advertise<uuv_guidance::GuidanceWaypoints>("/uuv_guidance/guidance_controller/waypoints", 1000);
    ros::Publisher  uuv_path = nh.advertise<nav_msgs::Path>("/uuv_planning/motion_planning/desired_path", 1000);

    ros::Subscriber trajectory_select = nh.subscribe("/uuv_planning/motion_planning/desired_trajectory",
                                                     1000,
                                                     &OnTrajectoryReceive);

    ros::Subscriber radius_select = nh.subscribe("/uuv_planning/motion_planning/circle_radius",
                                                     1000,
                                                     &OnRadiusReceive);

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        switch((int)trajectory_selector)
        {
            case 0:
                if (path_publish_flag == 0)
                {
                    uuv_guidance::GuidanceWaypoints waypoints = uuv_commons::GenerateCircle(circle_radius, 2, 5, 2);
                    waypoints.guidance_law = 1;

                    nav_msgs::Path path;
                    
                    path.header.stamp     = ros::Time::now();
                    path.header.frame_id  = "world";

                    for (int i = 0; i < waypoints.waypoint_list_length; i++)
                    {
                        geometry_msgs::PoseStamped      pose;

                        pose.header.stamp       = ros::Time::now();
                        pose.header.frame_id    = "world";
                        pose.pose.position.x    = waypoints.waypoint_list_x[i];
                        pose.pose.position.y    = -waypoints.waypoint_list_y[i];
                        pose.pose.position.z    = -waypoints.waypoint_list_z[i];

                        path.poses.push_back(pose);
                    }

                    /* Publish Odometry */ 
                    uuv_path.publish(path);
                    uuv_waypoints.publish(waypoints);

                    path_publish_flag = 1;
                }
                break;
            case 1:
                if (path_publish_flag == 0)
                {
                    uuv_guidance::GuidanceWaypoints waypoints = uuv_commons::GenerateCircle(circle_radius, 1.2, 2.7, 0);
                    waypoints.guidance_law = 2;

                    nav_msgs::Path path;
                    
                    path.header.stamp     = ros::Time::now();
                    path.header.frame_id  = "world";

                    for (int i = 0; i < waypoints.waypoint_list_length; i++)
                    {
                        geometry_msgs::PoseStamped      pose;

                        pose.header.stamp       = ros::Time::now();
                        pose.header.frame_id    = "world";
                        pose.pose.position.x    = waypoints.waypoint_list_x[i];
                        pose.pose.position.y    = -waypoints.waypoint_list_y[i];
                        pose.pose.position.z    = -waypoints.waypoint_list_z[i];

                        path.poses.push_back(pose);
                    }
                    
                    /* Publish Odometry */ 
                    uuv_path.publish(path);
                    uuv_waypoints.publish(waypoints);

                    path_publish_flag = 1;
                }
                break;
            case 2:
                if (path_publish_flag == 0)
                {
                    uuv_guidance::GuidanceWaypoints waypoints;

                    waypoints.guidance_law = 1;
                    waypoints.waypoint_list_length = 10;
                    waypoints.waypoint_list_x = {0,4,5,3,5,2,-3,-5,-2,0};
                    waypoints.waypoint_list_y = {0,4,2,-1,-3,-4,-3,0,4,0};
                    waypoints.waypoint_list_z = {0,2,0,-1,2,3,0.3,0.7,-1.4,0};
                    nav_msgs::Path path;
                    
                    path.header.stamp     = ros::Time::now();
                    path.header.frame_id  = "world";

                    for (int i = 0; i < waypoints.waypoint_list_length; i++)
                    {
                        geometry_msgs::PoseStamped      pose;

                        pose.header.stamp       = ros::Time::now();
                        pose.header.frame_id    = "world";
                        pose.pose.position.x    = waypoints.waypoint_list_x[i];
                        pose.pose.position.y    = -waypoints.waypoint_list_y[i];
                        pose.pose.position.z    = -waypoints.waypoint_list_z[i];

                        path.poses.push_back(pose);
                    }
                    
                    /* Publish Odometry */ 
                    uuv_path.publish(path);
                    uuv_waypoints.publish(waypoints);

                    path_publish_flag = 1;
                }
                break;
            default:
                break;
        }


        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}
