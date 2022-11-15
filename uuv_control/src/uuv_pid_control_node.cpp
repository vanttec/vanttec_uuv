/** ----------------------------------------------------------------------------
 * @file: uuv_control_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS control node for the UUV. Uses uuv_control library.
 * -----------------------------------------------------------------------------
 **/

#include "uuv_6dof_pid.hpp"
#include "vtec_u4_6dof_dynamic_model.hpp"
#include "vanttec_uuv/EtaPose.h"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_control_node");
    ros::NodeHandle nh;
    
    ros::Rate           cycle_rate(int(1 / SAMPLE_TIME_S));
    float k_p[6] = {8, 5, 7, 60, 40, 80};
    float k_i[6] = {0, 0, 0, 0, 0.01, 0.01};
    float k_d[6] = {1.5, 5, 0.7, 7, 5, 20};
    DOFControllerType_E types[6] = {LINEAR_DOF, LINEAR_DOF, LINEAR_DOF, ANGULAR_DOF, ANGULAR_DOF, ANGULAR_DOF};

    UUV6DOFPIDController   system_controller(SAMPLE_TIME_S, k_p, k_i, k_d, types);
    
    ros::Publisher  uuv_thrust      = nh.advertise<vanttec_uuv::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1000);
    ros::Subscriber uuv_dynamics    = nh.subscribe("/uuv_simulation/dynamic_model/non_linear_functions", 10, 
                                                    &UUV6DOFPIDController::UpdateDynamics,
                                                    &system_controller);

    ros::Subscriber uuv_pose        = nh.subscribe("/uuv_simulation/dynamic_model/eta_pose", 10,
                                                    &UUV6DOFPIDController::UpdatePose,
                                                    &system_controller);

    // ros::Subscriber uuv_twist       = nh.subscribe("/uuv_simulation/dynamic_model/vel", 
    //                                                 10,
    //                                                 &UUV6DOFPIDController::UpdateTwist,
    //                                                 &system_controller);

    ros::Subscriber uuv_setpoint    = nh.subscribe("/uuv_control/uuv_control_node/setpoint", 10,
                                                    &UUV6DOFPIDController::UpdateSetPoints,
                                                    &system_controller); 

    int counter = 0;
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* Update Parameters with new info */ 
        system_controller.CalculateManipulations();
       
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