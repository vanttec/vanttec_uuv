/** ----------------------------------------------------------------------------
 * @file: vtec_u4_pid_node.cpp
 * @date: May 6, 2023
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS control node for the UUV. Uses uuv_control library.
 * -----------------------------------------------------------------------------
 **/

#include "controllers/feedback_linearization/model_based_controllers/UUVs/vtec_u4_6dof_in_pid.hpp"

#include "vanttec_msgs/EtaPose.h"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_control_node");
    ros::NodeHandle private_nh("~");
    
    ros::Rate      cycle_rate(int(1 / SAMPLE_TIME_S));
    
    std::vector<float> k_p;
    std::vector<float> k_i;
    std::vector<float> k_d;

    std::array<float,6> chi1_d {1, 1, 0, 0, 0, 0};
    std::array<float,6> chi2_d {0, 0, 0, 0, 0, 0};
    std::array<float,6> chi2_dot_d {0, 0, 0, 0, 0, 0};

    std::vector<float> init_pose;

    vanttec_msgs::EtaPose q_d;

    double t_init;
    double t_cur;

    private_nh.getParam("k_p", k_p);
    private_nh.getParam("k_i", k_i);
    private_nh.getParam("k_d", k_d);
    private_nh.param("init_pose", init_pose, {0,0,0,0,0,0});

    std::array<float,6> U_MAX {127, 34, 118, 28, 9.6, 36.6};
    std::array<DOFControllerType_E,6> types {LINEAR_DOF, LINEAR_DOF, LINEAR_DOF, ANGULAR_DOF, ANGULAR_DOF, ANGULAR_DOF};

    VTEC_U4_6DOF_PID   model(SAMPLE_TIME_S, k_p, k_i, k_d, U_MAX, types);
    
    ros::Publisher  uuv_accel     = private_nh.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_acc", 10);
    ros::Publisher  uuv_vel       = private_nh.advertise<geometry_msgs::Twist>("/uuv_simulation/dynamic_model/vel", 10);
    ros::Publisher  uuv_eta_pose  = private_nh.advertise<vanttec_msgs::EtaPose>("/uuv_simulation/dynamic_model/eta_pose", 10);
    ros::Publisher  uuv_thrust    = private_nh.advertise<vanttec_msgs::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1);

    model.setInitPose(init_pose);
    model.updateReferences(chi1_d, chi2_d, chi2_dot_d);

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();
        
        /* calculate Model States */
        model.calculateStates();

        model.updateNonLinearFunctions();

        model.calculateControlSignals();

        model.updateControlSignals();

        /* Publish Odometry */
        uuv_accel.publish(model.accelerations_);
        uuv_vel.publish(model.velocities_);
        uuv_eta_pose.publish(model.eta_pose_);
       
        /* Publish Thrust */
        // uuv_thrust.publish(PID.thrust_);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}