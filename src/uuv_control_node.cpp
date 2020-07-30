#include "uuv_4dof_controller.hpp"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_control_node");
    ros::NodeHandle nh;
    
    ros::Rate           cycle_rate(int(1 / SAMPLE_TIME_S));
    UUV4DOFController   system_controller(SAMPLE_TIME_S, Kpid_u, Kpid_v, Kpid_z, Kpid_psi);
    
    ros::Publisher  uuv_thrust      = nh.advertise<vanttec_uuv::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1000);

    ros::Subscriber uuv_pose        = nh.subscribe("/uuv_simulation/dynamic_model/pose",
                                                    1000,
                                                    &UUV4DOFController::UpdatePose,
                                                    &system_controller);

    ros::Subscriber uuv_twist       = nh.subscribe("/uuv_simulation/dynamic_model/vel", 
                                                    10,
                                                    &UUV4DOFController::UpdateTwist,
                                                    &system_controller);

    ros::Subscriber uuv_setpoint    = nh.subscribe("/uuv_control/uuv_control_node/setpoint", 
                                                    10,
                                                    &UUV4DOFController::UpdateSetPoints,
                                                    &system_controller); 

    int counter = 0;
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* Update Parameters with new info */ 
        system_controller.UpdateControlLaw();
        system_controller.UpdateThrustOutput();
        
        /*
        if (counter % 10 == 0)
        {
            std::cout << "E: " << system_controller.heading_controller.error << std::endl;
            std::cout << "S: " << system_controller.heading_controller.set_point << std::endl;
        }

        counter++;     
        */
       
        /* Publish Odometry */ 
        uuv_thrust.publish(system_controller.thrust);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}