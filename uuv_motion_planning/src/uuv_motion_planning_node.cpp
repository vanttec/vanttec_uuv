#include <uuv_guidance/GuidanceWaypoints.h>

#include <ros/ros.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_motion_planning");
    ros::NodeHandle nh;
    
    ros::Rate       cycle_rate(int(1 / SAMPLE_TIME_S));
    
    ros::Publisher  uuv_waypoints = nh.advertise<uuv_guidance::GuidanceWaypoints>("/uuv_guidance/guidance_controller/waypoints", 1000);

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        uuv_guidance::GuidanceWaypoints waypoints;

        waypoints.guidance_law = 1;
        waypoints.waypoint_list_length = 5;
        waypoints.waypoint_list_x = {0,4,5,3,5};
        waypoints.waypoint_list_y = {0,4,2,-1,-3};
        waypoints.waypoint_list_z = {0,0,0,1,-2};

        /* Publish Odometry */ 
        uuv_waypoints.publish(waypoints);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}