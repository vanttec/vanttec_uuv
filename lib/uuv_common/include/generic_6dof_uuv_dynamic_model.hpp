/** ----------------------------------------------------------------------------
 * @file: generic_6dof_uuv_model.hpp
 * @date: March 20, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a generic 6dof UUV model.
 * -----------------------------------------------------------------------------
 **/
#ifndef __GENERIC_6DOF_UUV_DYNAMIC_MODEL__
#define __GENERIC_6DOF_UUV_DYNAMIC_MODEL__

#include "vanttec_uuv/ThrustControl.h"
#include "vanttec_uuv/EtaPose.h"
#include "uuv_common.hpp"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

class Generic6DOFUUVDynamicModel
{
    public:
        geometry_msgs::Vector3  linear_acceleration;
        geometry_msgs::Twist    velocities;
        vanttec_uuv::EtaPose    eta_pose;

        Generic6DOFUUVDynamicModel(float sample_time_s);
        ~Generic6DOFUUVDynamicModel();

        void ThrustCallback(const vanttec_uuv::ThrustControl& _thrust);
        void CalculateStates();

    private:
        Eigen::MatrixXf J;

        Eigen::VectorXf tau;

        Eigen::VectorXf upsilon;
        // Eigen::VectorXf upsilon_prev;
        Eigen::VectorXf upsilon_dot;
        Eigen::VectorXf upsilon_dot_prev;
        Eigen::VectorXf eta_dot;
        Eigen::VectorXf eta_dot_prev;
        Eigen::VectorXf eta; // x, y, z, phi, theta, psi
        // Eigen::Vector4f quat;

        Eigen::MatrixXf M_rb;
        Eigen::MatrixXf M_a;
        Eigen::MatrixXf C_rb;
        Eigen::MatrixXf C_a;
        Eigen::MatrixXf D_lin;
        Eigen::MatrixXf D_qua;
        Eigen::VectorXf G_eta;

        float _sample_time_s;

        /* Body Parameters */
        const float mass;
        const float volume;
        const float Ixx;
        const float Ixy;
        const float Ixz;
        const float Iyx;
        const float Iyy;
        const float Iyz;
        const float Izx;
        const float Izy;
        const float Izz;
        const float weight;
        const float buoyancy;
        const float x_b;
        const float y_b;
        const float z_b;

        /* Thruster angle parameters */
        const float beta;
        const float epsilon;
        const float rv_y;
        const float rh_x;
        const float rh_y;
        const float rh_z;
        const float b;
        const float l;

        /* Added Mass Parameters */
        const float X_u_dot;
        const float Y_v_dot;
        const float Z_w_dot;
        const float K_p_dot;
        const float M_q_dot;
        const float N_r_dot;

        /* Damping Parameters */
        const float X_u;
        const float Y_v;
        const float Z_w;
        const float K_p;
        const float M_q;
        const float N_r;

        const float X_uu;
        const float Y_vv;
        const float Z_ww;
        const float K_pp;
        const float M_qq;
        const float N_rr;

        /* Max Thrust Values for different DoFs */
        const float MAX_THRUST_PER_THRUSTER = 35;
        const float MAX_THRUST_SURGE = 100;
        const float MAX_THRUST_SWAY = 100;
        const float MAX_THRUST_HEAVE = 100;
        const float MAX_THRUST_YAW = 100;
}

#endif