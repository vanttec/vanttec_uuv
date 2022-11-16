/** ----------------------------------------------------------------------------
 * @file: uuv_waypoint_publisher_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS waypoint publisher node for the UUV. Uses uuv_motion_planning
 *         library.
 * -----------------------------------------------------------------------------
 **/

#include "waypoint_publisher.hpp"

#include <ros/ros.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_waypoint_publisher");
    ros::NodeHandle nh;
    
    ros::Rate           cycle_rate(int(1 / SAMPLE_TIME_S));
    WaypointPublisher   waypoint_publisher;
    
    ros::Publisher  uuv_waypoints = nh.advertise<vanttec_msgs::GuidanceWaypoints>("/uuv_guidance/guidance_controller/waypoints", 1000);
    ros::Publisher  uuv_path = nh.advertise<nav_msgs::Path>("/uuv_planning/motion_planning/desired_path", 1000);

    ros::Subscriber trajectory_select = nh.subscribe("/uuv_planning/motion_planning/desired_trajectory",
                                                     1000,
                                                     &WaypointPublisher::OnTrajectoryReceive,
                                                     &waypoint_publisher);

    ros::Subscriber radius_select = nh.subscribe("/uuv_planning/motion_planning/circle_radius",
                                                     1000,
                                                     &WaypointPublisher::OnRadiusReceive,
                                                     &waypoint_publisher); 

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        waypoint_publisher.WaypointSelection();

        if (waypoint_publisher.path_publish_flag == 0)
        {
            uuv_path.publish(waypoint_publisher.path);
            uuv_waypoints.publish(waypoint_publisher.waypoints);
            waypoint_publisher.path_publish_flag_ = 1;
        }
        
        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}
