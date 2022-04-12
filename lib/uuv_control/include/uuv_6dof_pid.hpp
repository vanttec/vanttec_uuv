/** ----------------------------------------------------------------------------
 * @file: uuv_6dof_pid.hpp
 * @date: April 10, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: 6-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_6DOF_PID_H__
#define __UUV_6DOF_PID_H__

#include "pid_controller.hpp"
#include "vanttec_uuv/ThrustControl.h"
#include "vanttec_uuv/SystemDynamics.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>

class UUV6DOFPIDController
{
    private:
        ros::NodeHandle handle;
        bool functs_arrived;
    public:
        Eigen::VectorXf u;              // Control
        Eigen::VectorXf ua;             // Auxiliary Control
        Eigen::VectorXf f;
        Eigen::MatrixXf g;
        Eigen::VectorXf ref_dot_dot;
        vanttec_uuv::ThrustControl  thrust;

        PIDController PID_x;
        PIDController PID_y;
        PIDController PID_z;
        PIDController PID_phi;
        PIDController PID_theta;
        PIDController PID_psi;

        UUV6DOFPIDController(const float &_sample_time_s, const float *_k_p, const float *_k_i, const float *_k_d, const DOFControllerType_E *_type);
        ~UUV6DOFPIDController();

        void CalculateManipulations();
        void UpdateDynamics(const vanttec_uuv::SystemDynamics& _non_linear_functions);
        void UpdateSetPoints(const vanttec_uuv::EtaPose& _set_points);
        void UpdatePose(const vanttec_uuv::EtaPose& _current);
};

#endif