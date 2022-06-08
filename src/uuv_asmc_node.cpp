/** ----------------------------------------------------------------------------
 * @file: uuv_asmc_node.cpp
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ROS adaptive sliding mode control node for the UUV. Uses uuv_control library.
 * -----------------------------------------------------------------------------
 **/

#include "uuv_6dof_asmc.hpp"
#include "vtec_u4_6dof_dynamic_model.hpp"
#include "vanttec_uuv/EtaPose.h"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_control_node");
    ros::NodeHandle nh;
    
    ros::Rate cycle_rate(int(1 / SAMPLE_TIME_S));

    // Sliding surface params
    float lambda[6] = {0.3, 0.3, 0.3, 0.8, 0.8, 0.8};
    
    // K2 gains
    // float K2[6] = {0.5, 0.5, 0.3, 0.3, 0.3, 0.3};
    float K2[6] = {1,1,1,1,1,1};
    
    // Adaptive law params
    // float K_alpha[6] = {1.2, 0.8, 0.5, 6, 0.7, 4};
    // float K_min[6] = {0.1, 0.1, 0.1, 0.2, 0.1, 0.2};
    // float mu[6] = {0.01, 0.01, 0.01, 0.005, 0.005, 0.005};
    
    float K_alpha[6] = {0.1,0.1,0.1,0.1,0.1,0.1};
    float K_min[6] = {0.01, 0.01, 0.01,0.01, 0.01, 0.01};
    float mu[6] = {0.01, 0.01, 0.01,0.01, 0.01, 0.01};

    // Max Tau
    float MAX_TAU[6] = {127, 34, 118, 28, 9.6, 36.6};

    UUV_6DOF_ASMC   system_controller(SAMPLE_TIME_S, lambda, K2, K_alpha, K_min, mu);
    system_controller.SetTauLimits(MAX_TAU);
    
    ros::Publisher  uuv_thrust      = nh.advertise<vanttec_uuv::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1000);
    ros::Subscriber uuv_dynamics    = nh.subscribe("/uuv_dynamics/non_linear_functions", 10, 
                                                    &UUV_6DOF_ASMC::UpdateDynamics,
                                                    &system_controller);

    ros::Subscriber uuv_pose        = nh.subscribe("/uuv_simulation/dynamic_model/eta_pose", 10,
                                                    &UUV_6DOF_ASMC::UpdatePose,
                                                    &system_controller);

    ros::Subscriber uuv_setpoint    = nh.subscribe("/uuv_control/uuv_control_node/setpoint", 10,
                                                    &UUV_6DOF_ASMC::UpdateSetPoints,
                                                    &system_controller); 

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* Update Parameters with new info */ 
        system_controller.CalculateManipulation();
       
        /* Publish Odometry */
        // Current way: if no functions arrive through the subscriber, the last computed thrusts are published.
        // An option could be to use ros::topic::waitForMessage to publish once the nonlinear functioncs arrive, in order to
        // avoid publishing garbage.
        uuv_thrust.publish(system_controller.thrust);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}