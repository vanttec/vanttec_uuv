/** ----------------------------------------------------------------------------
 * @file: uuv_guidance_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS guidance node for the UUV. Uses uuv_guidance library.
 * -----------------------------------------------------------------------------
 **/

#include <uuv_guidance_controller.hpp>

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_guidance_node");
    ros::NodeHandle nh;
    
    ros::Rate               cycle_rate(int(1 / SAMPLE_TIME_S));
    GuidanceController      guidance_controller(SAMPLE_TIME_S);
    
    ros::Publisher  uuv_desired_setpoints       = nh.advertise<geometry_msgs::Twist>("/uuv_control/uuv_control_node/setpoint", 1000);

    ros::Subscriber uuv_pose                    = nh.subscribe("/uuv_simulation/dynamic_model/pose",
                                                                1000,
                                                                &GuidanceController::OnCurrentPositionReception,
                                                                &guidance_controller);

    ros::Subscriber uuv_e_stop                  = nh.subscribe("/uuv_master/uuv_master_node/e_stop",
                                                                1000,
                                                                &GuidanceController::OnEmergencyStop,
                                                                &guidance_controller);

    ros::Subscriber uuv_waypoints               = nh.subscribe("/uuv_guidance/guidance_controller/waypoints",
                                                                1000,
                                                                &GuidanceController::OnWaypointReception,
                                                                &guidance_controller);

    ros::Subscriber uuv_status                  = nh.subscribe("/uuv_master/uuv_master_node/status",
                                                                1000,
                                                                &GuidanceController::OnMasterStatus,
                                                                &guidance_controller);

    uint32_t counter = 0;
 
    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* Update Parameters with new info */ 
        guidance_controller.UpdateStateMachines();

        /* Publish Odometry */ 
        if (guidance_controller.uuv_status.status == 1)
        {
            uuv_desired_setpoints.publish(guidance_controller.desired_setpoints);
        }

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}