/** ----------------------------------------------------------------------------
 * @file: 6dof_vtec_u4_pid_test.cpp
 * @date: August 13, 2022
 * @author: Sebastian Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: test file for a 6DOF PID controller for the VTec U4.
 * -----------------------------------------------------------------------------
**/

#include "6dof_pid.hpp"
#include "vtec_u4_6dof_dynamic_model.hpp"
#include "vanttec_msgs/EtaPose.h"

#include <ros/ros.h>
#include <stdio.h>
#include <gtest/gtest.h>

const float SAMPLE_TIME_S = 0.01;

// Declare a test
TEST(ControlSuite, regulationObjective)
{
    ros::Rate           cycle_rate(int(1 / SAMPLE_TIME_S));
    float k_p[6] = {8, 5, 7, 60, 40, 80};
    float k_i[6] = {0, 0, 0, 0, 0.01, 0.01};
    float k_d[6] = {1.5, 5, 0.7, 7, 5, 20};
    DOFControllerType_E types[6] = {LINEAR_DOF, LINEAR_DOF, LINEAR_DOF, ANGULAR_DOF, ANGULAR_DOF, ANGULAR_DOF};

    SixDOFPID   system_controller(SAMPLE_TIME_S, k_p, k_i, k_d, types);
    
    ros::Publisher  uuv_thrust      = nh.advertise<vanttec_msgs::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1000);
    ros::Subscriber uuv_dynamics    = nh.subscribe("/uuv_simulation/dynamic_model/non_linear_functions", 10, 
                                                    &SixDOFPID::updateDynamics,
                                                    &system_controller);

    ros::Subscriber uuv_pose        = nh.subscribe("/uuv_simulation/dynamic_model/eta_pose_", 10,
                                                    &SixDOFPID::updatePose,
                                                    &system_controller);

    // ros::Subscriber uuv_twist       = nh.subscribe("/uuv_simulation/dynamic_model/vel", 
    //                                                 10,
    //                                                 &SixDOFPID::updateTwist,
    //                                                 &system_controller);

    ros::Subscriber uuv_set_point    = nh.subscribe("/uuv_control/uuv_control_node/set_point", 10,
                                                    &SixDOFPID::updateSetPoints,
                                                    &system_controller); 

    int counter = 0;
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* update Parameters with new info */ 
        system_controller.calculateManipulations();
       
        /* Publish Odometry */
        // Current way: if no functions arrive through the subscriber, the last computed thrusts are published.
        // An option could be to use ros::topic::waitForMessage to publish once the nonlinear functioncs arrive, in order to
        // avoid publishing garbage.
        uuv_thrust.publish(system_controller.thrust_);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "6dof_vtec_u4_pid_test");
    ros::NodeHandle nh;
    
      
    return RUN_ALL_TESTS();
}