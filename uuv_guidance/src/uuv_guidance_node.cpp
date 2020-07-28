#include <uuv_guidance_controller.hpp>

#include <ros/ros.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_guidance_node");
    ros::NodeHandle nh;
    
    ros::Rate               cycle_rate(int(1 / SAMPLE_TIME_S));
    GuidanceController      guidance_controller;
    
    ros::Publisher  uuv_desired_setpoints       = nh.advertise<geometry_msgs::Twist>("/uuv_control/uuv_control_node/setpoint", 1000);

    ros::Subscriber uuv_pose                    = nh.subscribe("/uuv_simulation/dynamic_model/pose",
                                                                1000,
                                                                &GuidanceController::OnCurrentPositionReception,
                                                                &guidance_controller);

    ros::Subscriber uuv_e_stop                  = nh.subscribe("/uuv_guidance/guidance_controller/e_stop",
                                                                1000,
                                                                &GuidanceController::OnEmergencyStop,
                                                                &guidance_controller);

    ros::Subscriber uuv_waypoints               = nh.subscribe("/uuv_guidance/guidance_controller/waypoints",
                                                                1000,
                                                                &GuidanceController::OnWaypointReception,
                                                                &guidance_controller);

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* Update Parameters with new info */ 
        guidance_controller.UpdateStateMachines();

        /* Publish Odometry */ 
        uuv_desired_setpoints.publish(guidance_controller.desired_setpoints);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}