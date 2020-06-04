#ifndef __UUV_DYNAMIC_4DOF_MODEL_H__
#define __UUV_DYNAMIC_4DOF_MODEL_H__

#include "uuv_control/ThrustControl.h"
#include "vtec_u3_gamma_parameters.hpp"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>

class UUVDynamic4DOFModel
{
    public:

        float sample_time_s;

        geometry_msgs::Vector3  linear_acceleration;
        geometry_msgs::Vector3  angular_rate;
        geometry_msgs::Vector3  angular_position;

        geometry_msgs::Twist    velocities;
        geometry_msgs::Pose     pose;

        UUVDynamic4DOFModel(float _sample_time_s);
        ~UUVDynamic4DOFModel();

        void ThrustCallback(const uuv_control::ThrustControl& _thrust);
        void CalculateStates();
    
    private:

        /* Matrices */
        
        Eigen::Vector4f tau;
        Eigen::Vector4f body_pos;
        Eigen::Vector4f upsilon;
        Eigen::Vector4f upsilon_prev;
        Eigen::Vector4f upsilon_dot;
        Eigen::Vector4f upsilon_dot_prev;
        Eigen::Matrix4f M_rb;
        Eigen::Matrix4f M_a;
        Eigen::Matrix4f C_rb;
        Eigen::Matrix4f C_a;
        Eigen::Matrix4f D_lin;
        Eigen::Matrix4f D_qua;
        Eigen::Vector4f G_eta;
        Eigen::Matrix4f J;
        Eigen::Vector4f eta_dot;
};

#endif