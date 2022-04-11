/** ----------------------------------------------------------------------------
 * @file: uuv_4dof_pid_controller.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: 4-DOF controller class, using a different, decoupled controller for
 *         each DOF.
           OUTDATED.
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_4DOF_PID_CONTROLLER_H__
#define __UUV_4DOF_PID_CONTROLLER_H__

#include "pid_controller.hpp"
#include "vtec_u3_parameters.hpp"
#include "vanttec_uuv/ThrustControl.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>

class UUV4DOFController
{
    public:
        geometry_msgs::Pose         local_pose;
        geometry_msgs::Twist        local_twist;
        
        vanttec_uuv::ThrustControl  thrust;

        float yaw_psi_angle;

        PIDController surge_speed_controller;
        PIDController sway_speed_controller;
        PIDController depth_controller;
        PIDController heading_controller;

        Eigen::Vector4f f_x;
        Eigen::Vector4f g_x;

        UUV4DOFController(float _sample_time_s, const float _kpid_u[3], const float _kpid_v[3], const float _kpid_z[3], const float _kpid_psi[3]);
        ~UUV4DOFController();

        void UpdatePose(const geometry_msgs::Pose& _pose);
        void UpdateTwist(const geometry_msgs::Twist& _twist);
        void UpdateSetPoints(const geometry_msgs::Twist& _set_points);
        
        void UpdateControlLaw();
        void UpdateThrustOutput();

        void PublishAccel();
    
    private:

        Eigen::Vector4f upsilon;
        Eigen::Matrix4f M_rb;
        Eigen::Matrix4f M_a;
        Eigen::Matrix4f C_rb;
        Eigen::Matrix4f C_a;
        Eigen::Matrix4f D_lin;
        Eigen::Matrix4f D_qua;
        Eigen::Vector4f G_eta;

        ros::NodeHandle handle;
        ros::Publisher v_dot_pub = handle.advertise<geometry_msgs::Twist>("/uuv_accel",1000);
};

#endif