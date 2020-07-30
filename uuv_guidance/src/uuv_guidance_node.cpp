#include <uuv_guidance_controller.hpp>

#include <ros/ros.h>

#include <stdio.h>

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

    uint32_t counter = 0;

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* Update Parameters with new info */ 
        guidance_controller.UpdateStateMachines();

        if (counter % 100 == 0)
        {
            std::cout << guidance_controller.current_guidance_law << std::endl;
        }

        /*
        if (counter % 10 == 0)
        {
            std::cout << "dist: " << guidance_controller.euclidean_distance << std::endl;

            if (guidance_controller.los_state_machine.state_machine != 0)
            {
                std::cout << "wp: " << guidance_controller.los_state_machine.current_waypoint << std::endl;
                std::cout << "xk1: " << guidance_controller.current_waypoint_list.waypoint_list_x[guidance_controller.los_state_machine.current_waypoint + 1] << std::endl;
                std::cout << "yk1: " << guidance_controller.current_waypoint_list.waypoint_list_y[guidance_controller.los_state_machine.current_waypoint + 1] << std::endl;
                std::cout << "x: " << guidance_controller.current_positions_ned.position.x << std::endl;
                std::cout << "y: " << guidance_controller.current_positions_ned.position.y << std::endl;
            }
        }
        
        counter++;
        */

        /* Publish Odometry */ 
        uuv_desired_setpoints.publish(guidance_controller.desired_setpoints);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}