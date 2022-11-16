/** ----------------------------------------------------------------------------
 * @file: vtec_u4_asmc_node.cpp
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ROS adaptive sliding mode control node for the VTec U4.
 * -----------------------------------------------------------------------------
 **/

#include "6dof_asmc.hpp"
#include "vanttec_msgs/EtaPose.h"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_control_node");
    ros::NodeHandle nh("~");
    
    ros::Rate cycle_rate(int(1 / SAMPLE_TIME_S));

    std::vector<float> lambda_v;
    std::vector<float> K2_v;
    std::vector<float> K1_init_v;
    std::vector<float> K_alpha_v;
    std::vector<float> K_min_v;
    std::vector<float> mu_v;

    // Sliding surface params
    nh.getParam("lambda", lambda_v);

    // K1 and K2 gains
    nh.getParam("K1_init", K1_init_v);
    nh.getParam("K2", K2_v);
    
    // Adaptive law params
    nh.getParam("K_alpha", K_alpha_v);
    nh.getParam("K_min", K_min_v);
    nh.getParam("mu", mu_v);

    // Max Tau
    float MAX_TAU[6] = {127, 34, 118, 28, 9.6, 36.6};

    float* lambda = &lambda_v[0];
    float* K1_init = &K1_init_v[0];
    float* K2 = &K2_v[0];
    float* K_alpha = &K_alpha_v[0];
    float* K_min = &K_min_v[0];
    float* mu = &mu_v[0];

    ASMC6DOF   system_controller(SAMPLE_TIME_S, lambda, K2, K_alpha, K1_init, K_min, mu);
    system_controller.setMaxThrust(MAX_TAU);
    
    ros::Publisher  uuv_thrust      = nh.advertise<vanttec_msgs::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1);
    ros::Subscriber uuv_dynamics    = nh.subscribe("/uuv_simulation/dynamic_model/non_linear_functions", 1, 
                                                    &ASMC6DOF::updateDynamics,
                                                    &system_controller);

    ros::Subscriber uuv_pose        = nh.subscribe("/uuv_simulation/dynamic_model/eta_pose_", 1,
                                                    &ASMC6DOF::updatePose,
                                                    &system_controller);

    ros::Subscriber uuv_set_point    = nh.subscribe("/uuv_control/uuv_control_node/set_point", 1,
                                                    &ASMC6DOF::updateSetPoints,
                                                    &system_controller); 

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* update Parameters with new info */ 
        system_controller.calculateManipulation();
       
        /* Publish Odometry */
        // Current way: if no functions arrive through the subscriber, the last computed thrusts are published.
        // An option could be to use ros::topic::waitForMessage to publish once the nonlinear functioncs arrive, in order to
        // avoid publishing garbage.
        uuv_thrust.publish(system_controller.thrust_);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}