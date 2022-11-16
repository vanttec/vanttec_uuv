/** ----------------------------------------------------------------------------
 * @file: uuv_control_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS control node for the UUV. Uses uuv_control library.
 * -----------------------------------------------------------------------------
 **/

#include "6dof_pid.hpp"
#include "vanttec_msgs/EtaPose.h"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_control_node");
    ros::NodeHandle nh("~");
    
    ros::Rate      cycle_rate(int(1 / SAMPLE_TIME_S));
    
    std::vector<float> g_p;
    std::vector<float> g_i;
    std::vector<float> g_d;

    vanttec_msgs::EtaPose q_d;

    double t_init;
    double t_cur;

    nh.getParam("k_p", g_p);
    nh.getParam("k_i", g_i);
    nh.getParam("k_d", g_d);

    float* k_p = &g_p[0];
    float* k_i = &g_i[0];
    float* k_d= &g_d[0];

    float MAX_TAU[6] = {127, 34, 118, 28, 9.6, 36.6};

    DOFControllerType_E types[6] = {LINEAR_DOF, LINEAR_DOF, LINEAR_DOF, ANGULAR_DOF, ANGULAR_DOF, ANGULAR_DOF};

    PID6DOF   system_controller(SAMPLE_TIME_S, k_p, k_i, k_d, types);
    system_controller.setMaxThrust(MAX_TAU);
    
    ros::Publisher  uuv_thrust      = nh.advertise<vanttec_msgs::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1);
    ros::Subscriber uuv_dynamics    = nh.subscribe("/uuv_simulation/dynamic_model/non_linear_functions", 1, 
                                                    &PID6DOF::updateDynamics,
                                                    &system_controller);

    ros::Subscriber uuv_pose        = nh.subscribe("/uuv_simulation/dynamic_model/eta_pose_", 1,
                                                    &PID6DOF::updatePose,
                                                    &system_controller);

    ros::Subscriber uuv_set_point    = nh.subscribe("/uuv_control/uuv_control_node/set_point", 1,
                                                    &PID6DOF::updateSetPoints,
                                                    &system_controller); 

    t_init = ros::Time::now().toSec();
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();
        t_cur = t_init - ros::Time::now().toSec();

        /* update Trajectory */
        q_d.x = 2*std::sin(t_cur/4) + 0.5;
        q_d.y = 2*std::cos(t_cur/4);
        q_d.z = 2*std::cos(t_cur/80 + 0.5);
        q_d.phi = 0.0;
        q_d.theta = 0.0;
        q_d.psi = 0.0;

        /* update Parameters with new info */ 
        system_controller.updateSetPoints(q_d);
        system_controller.calculateManipulations();
       
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