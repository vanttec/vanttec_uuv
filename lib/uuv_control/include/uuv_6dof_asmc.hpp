/** ----------------------------------------------------------------------------
 * @file: uuv_6dof_controller.hpp
 * @date: March 2, 2022
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF adaptive sliding mode controller class
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_6DOF_ASMC_H__
#define __UUV_6DOF_ASMC_H__

#include "asmc.hpp"
#include "vanttec_uuv/ThrustControl.h"
#include "vanttec_uuv/SystemDynamics.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>

class UUV_6DOF_ASMC
{
    private:
        ros::NodeHandle handle;
        // bool functs_arrived;

        float MAX_FORCE_X;
        float MAX_FORCE_Y;
        float MAX_FORCE_Z;
        float MAX_TORQUE_K;
        float MAX_TORQUE_M;
        float MAX_TORQUE_N; 
        
    public:
        Eigen::VectorXf u;              // Control
        Eigen::VectorXf ua;             // Auxiliary Control
        Eigen::VectorXf f;
        Eigen::MatrixXf g;
        Eigen::VectorXf q_dot_dot;      //ref_dot_dot
        vanttec_uuv::ThrustControl  thrust;

        ASMC ASMC_x;
        ASMC ASMC_y;
        ASMC ASMC_z;
        ASMC ASMC_phi;
        ASMC ASMC_theta;
        ASMC ASMC_psi;
        
        UUV_6DOF_ASMC(const float sample_time_s, const float lambda[6], const float K2[6], const float K_alpha[6],  const float K_min[6],  const float mu[6]);
        ~UUV_6DOF_ASMC();

        void SetTauLimits(const float* MAX_TAU);
        void UpdateDynamics(const vanttec_uuv::SystemDynamics& non_linear_functions);
        void UpdatePose(const vanttec_uuv::EtaPose& current);
        void UpdateSetPoints(const vanttec_uuv::EtaPose& set_points);
        void CalculateManipulation();

        // void UpdateTwist(const geometry_msgs::Twist& twist);
        
        // void UpdateControlLaw();
        // void UpdateThrustOutput();

        // void PublishAccel();
};

#endif