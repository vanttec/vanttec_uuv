/** ----------------------------------------------------------------------------
 * @file: uuv_dynamic_4dof_model.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Implementation of the kinematic 4dof model of the UUV for simulation.
 * -----------------------------------------------------------------------------
 **/

#ifndef __UUV_DYNAMIC_4DOF_MODEL_H__
#define __UUV_DYNAMIC_4DOF_MODEL_H__

#include "vanttec_uuv/ThrustControl.h"
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

        void ThrustCallback(const vanttec_uuv::ThrustControl& _thrust);
        void CalculateStates();
    
    private:

        /* Matrices */
        
        Eigen::VectorXf tau;
        Eigen::VectorXf body_pos;
        Eigen::VectorXf upsilon;
        Eigen::VectorXf upsilon_prev;
        Eigen::VectorXf upsilon_dot;
        Eigen::VectorXf upsilon_dot_prev;
        Eigen::MatrixXf M_rb;
        Eigen::MatrixXf M_a;
        Eigen::MatrixXf C_rb;
        Eigen::MatrixXf C_a;
        Eigen::MatrixXf D_lin;
        Eigen::MatrixXf D_qua;
        Eigen::VectorXf G_eta;  
        Eigen::Matrix3f R;
        Eigen::Matrix3f T;
        Eigen::MatrixXf J;
        Eigen::VectorXf eta_dot;
        Eigen::VectorXf eta;
        
        Eigen::Matrix3f zero;
        // Eigen::Vector4f quat;
};

#endif