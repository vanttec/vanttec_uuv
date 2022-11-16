/** ----------------------------------------------------------------------------
 * @file: vtec_u4_antsmc_node.cpp
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ROS adaptive non-singular terminal sliding mode control node for the VTec U4.
 * -----------------------------------------------------------------------------
 **/

#include "6dof_antsmc.hpp"
#include "vanttec_msgs/EtaPose.h"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_control_node");
    ros::NodeHandle nh("~");
    
    ros::Rate cycle_rate(int(1 / SAMPLE_TIME_S));

    std::vector<float> alpha_v;
    std::vector<float> beta_v;
    std::vector<float> K2_v;
    std::vector<float> K_alpha_v;
    std::vector<float> K1_init_v;
    std::vector<float> K_min_v;
    std::vector<float> mu_v;

    vanttec_msgs::EtaPose q_d;

    double t_init;
    double t_cur;

    // Sliding surface params
    nh.getParam("alpha", alpha_v);
    nh.getParam("beta_", beta_v);

    // K2 gains
    nh.getParam("K2", K2_v);
    
    // Adaptive law params
    nh.getParam("K_alpha", K_alpha_v);
    nh.getParam("K1_init", K1_init_v);
    nh.getParam("K_min", K_min_v);
    nh.getParam("mu", mu_v);

    // Max Tau
    float MAX_TAU[6] = {127, 34, 118, 28, 9.6, 36.6};

    float* alpha = &alpha_v[0];
    float* beta_ = &beta_v[0];
    float* K2 = &K2_v[0];
    float* K_alpha = &K_alpha_v[0];
    float* K1_init = &K1_init_v[0];
    float* K_min = &K_min_v[0];
    float* mu = &mu_v[0];

    ANTSMC6DOF   system_controller(SAMPLE_TIME_S, alpha, beta_, K2, K_alpha, K_min, K1_init, mu);
    system_controller.setMaxThrust(MAX_TAU);
    
    ros::Publisher  uuv_thrust      = nh.advertise<vanttec_msgs::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1);
    ros::Subscriber uuv_dynamics    = nh.subscribe("/uuv_simulation/dynamic_model/non_linear_functions", 1, 
                                                    &ANTSMC6DOF::updateDynamics,
                                                    &system_controller);

    ros::Subscriber uuv_pose        = nh.subscribe("/uuv_simulation/dynamic_model/eta_pose_", 1,
                                                    &ANTSMC6DOF::updatePose,
                                                    &system_controller);

    ros::Subscriber uuv_set_point    = nh.subscribe("/uuv_control/uuv_control_node/set_point", 1,
                                                    &ANTSMC6DOF::updateSetPoints,
                                                    &system_controller); 

    t_init = ros::Time::now().toSec();

    while(ros::ok())
    {
        t_cur = ros::Time::now().toSec() - t_init;
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* update Trajectory */
        // q_d.x = 2*std::sin(t_cur/4) + 0.5;
        // q_d.y = 2*std::cos(t_cur/4);
        // q_d.z = 2*std::cos(t_cur/80 + 0.5);
        // q_d.phi = 0.0;
        // q_d.theta = 0.0;
        // q_d.psi = 0.0;

        // system_controller.updateSetPoints(q_d);
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