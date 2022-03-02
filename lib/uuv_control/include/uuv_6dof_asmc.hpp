/** ----------------------------------------------------------------------------
 * @file: uuv_6dof_controller.hpp
 * @date: March 2, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF adaptive sliding mode controller class
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_6DOF_ASMC_H__
#define __UUV_6DOF_ASMC_H__

#include "asmc.hpp"
#include "vtec_u3_gamma_parameters.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>

class UUV_6DOF_ASMC
{
    private:
        Eigen::Vector6f upsilon;
        Eigen::Matrix6f M_rb;
        Eigen::Matrix6f M_a;
        Eigen::Matrix6f C_rb;
        Eigen::Matrix6f C_a;
        Eigen::Matrix6f D_lin;
        Eigen::Matrix6f D_qua;
        Eigen::Vector6f G_eta;

        ros::NodeHandle handle;
        ros::Publisher v_dot_pub = handle.advertise<geometry_msgs::Twist>("/uuv_accel",1000);
    public:
        geometry_msgs::Pose         local_pose;
        geometry_msgs::Twist        local_twist;
        
        vanttec_uuv::ThrustControl  thrust;

        float yaw_psi_angle;

        ASMC x_controller;
        ASMC y_controller;
        ASMC z_controller;
        ASMC phi_controller;
        ASMC theta_controller;
        ASMC psi_controller;

        Eigen::Vector6f f_x;
        Eigen::Vector6f g_x;

        UUV_6DOF_ASMC(float _sample_time_s, const float _kpid_u[3], const float _kpid_v[3], const float _kpid_z[3], const float _kpid_psi[3]);
        ~UUV_6DOF_ASMC();

        void UpdatePose(const geometry_msgs::Pose& _pose);
        void UpdateTwist(const geometry_msgs::Twist& _twist);
        void UpdateSetPoints(const geometry_msgs::Twist& _set_points);
        
        void UpdateControlLaw();
        void UpdateThrustOutput();

        void PublishAccel();
};

#endif __UUV_6DOF_ASMC_H__