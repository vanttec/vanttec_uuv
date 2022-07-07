/** ----------------------------------------------------------------------------
 * @file: generic_6dof_uuv_model.hpp
 * @date: March 20, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a generic 6dof UUV model in the non-inertial frame with
           Euler Angles.
   @todo: Modify matrices for the true general case of non-diagonal matrices.
          Include ALL terms. Also, include offset vector in the case the origin
          is no the COM.
 * -----------------------------------------------------------------------------
 **/
#ifndef __GENERIC_6DOF_UUV_DYNAMIC_MODEL__
#define __GENERIC_6DOF_UUV_DYNAMIC_MODEL__

#include "vanttec_uuv/ThrustControl.h"
#include "vanttec_uuv/EtaPose.h"
#include "uuv_common.hpp"

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

class Generic6DOFUUVDynamicModel
{
    private:
        float _sample_time_s;

        /* Transformation matrix */

        Eigen::MatrixXf J;
        Eigen::Matrix3f R;
        Eigen::Matrix3f T;

        /* System states */

        Eigen::VectorXf eta;            // x, y, z, phi, theta, psi
        Eigen::VectorXf eta_dot;
        Eigen::VectorXf eta_dot_prev;
        Eigen::VectorXf nu;             // u, v, w, p, q, r
        Eigen::VectorXf nu_dot;
        Eigen::VectorXf nu_dot_prev;
        // Eigen::VectorXf f;
        // Eigen::MatrixXf g;

        /* System matrices */
        
        Eigen::MatrixXf M;
        Eigen::MatrixXf M_rb;
        Eigen::MatrixXf M_a;
        Eigen::MatrixXf C;
        Eigen::MatrixXf C_rb;
        Eigen::MatrixXf C_a;
        Eigen::MatrixXf D;
        Eigen::MatrixXf D_lin;
        Eigen::MatrixXf D_qua;
        Eigen::VectorXf g_eta;

        /* Physical Parameters */

        float m;
        float W;
        float volume;
        float B;
        float Ixx;
        float Ixy;
        float Ixz;
        float Iyx;
        float Iyy;
        float Iyz;
        float Izx;
        float Izy;
        float Izz;

        /* Added Mass Parameters */

        float X_u_dot;
        float Y_v_dot;
        float Z_w_dot;
        float K_p_dot;
        float M_q_dot;
        float N_r_dot;

        /* Damping Parameters */

        float X_u;
        float Y_v;
        float Z_w;
        float K_p;
        float M_q;    
        float N_r;

        float X_uu;
        float Y_vv;
        float Z_ww;
        float K_pp;
        float M_qq;
        float N_rr;

        /* Distance from origin to center of mass */
        // They are the same

        /* Distance from origin to center of buoyancy  */

        float rb_x;
        float rb_y;
        float rb_z;

        /* Input forces vector */

        Eigen::VectorXf tau;
        Eigen::VectorXf u;

        float MAX_FORCE_X;
        float MAX_FORCE_Y;
        float MAX_FORCE_Z;
        float MAX_TORQUE_K;
        float MAX_TORQUE_M;
        float MAX_TORQUE_N;

    public:
        Eigen::VectorXf f;
        Eigen::MatrixXf g;
        
        vanttec_uuv::EtaPose    eta_pose;
        geometry_msgs::Twist    velocities;
        geometry_msgs::Accel    accelerations;

        Generic6DOFUUVDynamicModel(float sample_time_s);
        ~Generic6DOFUUVDynamicModel();

        void CalculateTransformation();
        virtual void CalculateCoriolis();
        virtual void CalculateDamping();
        void ThrustCallback(const vanttec_uuv::ThrustControl& _thrust);
        void CalculateStates();
        friend class VTecU4DynamicModel;
};

#endif