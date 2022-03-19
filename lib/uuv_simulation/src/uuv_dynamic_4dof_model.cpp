/** ----------------------------------------------------------------------------
 * @file: uuv_dynamic_4dof_model.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Implementation of the kinematic 4dof model of the UUV for simulation.
 * -----------------------------------------------------------------------------
 **/

#include "uuv_dynamic_4dof_model.hpp"

#include <math.h>
#include <stdio.h>
#include <iostream>

UUVDynamic4DOFModel::UUVDynamic4DOFModel(float _sample_time_s)
{
    this->sample_time_s = _sample_time_s;

    this->upsilon << 0,
                     0,
                     0,
                     0,
                     0,
                     0;
    
    this->upsilon_prev << 0,
                          0,
                          0,
                          0,
                          0,
                          0;
    
    this->upsilon_dot << 0,
                         0,
                         0,
                         0,
                         0,
                         0;
    
    this->upsilon_dot_prev << 0,
                              0,
                              0,
                              0,
                              0,
                              0;

    this->body_pos << 0,
                      0,
                      0,
                      0,
                      0,
                      0;

    this->tau << 0,
                 0,
                 0,
                 0,
                 0,
                 0;

    this->linear_acceleration.x = 0;
    this->linear_acceleration.y = 0;
    this->linear_acceleration.z = 0;

    this->angular_rate.x = 0;
    this->angular_rate.y = 0;
    this->angular_rate.z = 0;

    this->angular_position.x = 0;
    this->angular_position.y = 0;
    this->angular_position.z = 0;

    this->velocities.linear.x = 0;
    this->velocities.linear.y = 0;
    this->velocities.linear.z = 0;
    
    this->velocities.angular.x = 0;
    this->velocities.angular.y = 0;
    this->velocities.angular.z = 0;

    this->zero << 0,0,0,
                  0,0,0,
                  0,0,0;

}

UUVDynamic4DOFModel::~UUVDynamic4DOFModel(){}

void UUVDynamic4DOFModel::ThrustCallback(const vanttec_uuv::ThrustControl& _thrust)
{
    this->tau << _thrust.tau_x,
                 _thrust.tau_y,
                 _thrust.tau_z,
                 _thrust.tau_phi,
                 _thrust.tau_theta,
                 _thrust.tau_yaw;
}

void UUVDynamic4DOFModel::CalculateStates()
{
    this->upsilon_dot_prev = this->upsilon_dot;
    this->upsilon_prev = this->upsilon;

    /* Rigid Body Mass Matrix */

    this->M_rb << mass, 0, 0, 0, 0, 0,
                  0, mass, 0, 0, 0, 0,
                  0, 0, mass, 0, 0, 0,
                  0, 0, 0, Ixx, 0, 0,
                  0, 0, 0, 0, Iyy, 0,
                  0, 0, 0, 0, 0, Izz;

    /* Hydrodynamic Added Mass Matrix */

    this->M_a << X_u_dot, 0, 0, 0, 0, 0,
                 0, Y_v_dot, 0, 0, 0, 0,
                 0, 0, Z_w_dot, 0, 0, 0,
                 0, 0, 0, K_p_dot, 0, 0,
                 0, 0, 0, 0, M_q_dot, 0,
                 0, 0, 0, 0, 0, N_r_dot;

    /* Rigid Body Coriolis Matrix */

    float m_u = mass * this->upsilon(0);
    float m_v = mass * this->upsilon(1);
    float m_w = mass * this->upsilon(2);
    float ixx_p = Ixx * this->upsilon(3);
    float iyy_q = Iyy * this->upsilon(4);
    float izz_r = Izz * this->upsilon(3);
    
    this->C_rb << 0,   0,   0,   0,    m_w,    -m_v,
                  0,   0,   0,   -m_w,  0,     m_u,
                  0,   0,   0,   m_v,   -m_u,   0,
                  0,   m_w,  -m_v, 0,    -izz_r, iyy_q,
                  -m_w, 0 ,  m_u,  izz_r, 0,     -ixx_p,
                  m_v,  -m_u, 0,   -iyy_q, ixx_p,  0;

    /* Hydrodynamic Added Mass Coriolis Matrix */

    float a1 = X_u_dot * this->upsilon(0);
    float a2 = Y_v_dot * this->upsilon(1);
    float a3 = Z_w_dot * this->upsilon(2);
    float a4 = K_p_dot * this->upsilon(3);
    float a5 = M_q_dot * this->upsilon(4);
    float a6 = N_r_dot * this->upsilon(5);
    
    this->C_a << 0,     0,    0,    0,  -a3,  a2,
                 0,     0,    0,   a3,   0,  -a1,
                 0,     0,    0,  -a2,  a1,   0,
                 0,   -a3,  a2,   0,  -a6,  a5,
                 a3,   0,  -a1,  a6,   0,  -a4,
                -a2,  a1,   0,  -a5,   a4,  0;
    
    /* Hydrodynamic Damping */

    this->D_lin << -(X_u), 0,      0,    0,   0,    0,
                      0,   -(Y_v), 0,    0,   0,    0,
                      0,   0,    -(Z_w), 0,   0,    0,
                      0,   0,      0, -K_p,   0,    0,
                      0,   0,      0,    0, -M_q,   0,
                      0,   0,      0,    0,   0, -(N_r);

    this->D_qua << -(X_uu * fabs(this->upsilon(0))), 0, 0, 0, 0, 0,
                   0, -(Y_vv * fabs(this->upsilon(1))), 0, 0, 0, 0,
                   0, 0, -(Z_ww * fabs(this->upsilon(2))), 0, 0, 0,
                   0, 0, 0, -(K_pp * fabs(this->upsilon(3))), 0, 0,
                   0, 0, 0, 0, -(M_qq * fabs(this->upsilon(4))), 0,
                   0, 0, 0, 0, 0, -(N_rr * fabs(this->upsilon(5)));

    /* Restoring Forces */
    
    this->G_eta <<  (weight - buoyancy)*sin(body_pos(4)),
                   -(weight - buoyancy)*cos(body_pos(4))*sin(body_pos(3)),
                   -(weight - buoyancy)*cos(body_pos(4))*cos(body_pos(3)),
                   y_b*buoyancy*cos(body_pos(4))*cos(body_pos(3)) - z_b*buoyancy*cos(body_pos(4))*sin(body_pos(3)),
                   -z_b*buoyancy*sin(body_pos(3)) - x_b*buoyancy*cos(body_pos(4))*cos(body_pos(3)),
                   x_b*cos(body_pos(4))*sin(body_pos(3)) + y_b*buoyancy*sin(body_pos(4));

    /* 6 DoF State Calculation */

    Eigen::MatrixXf M = this->M_rb - this->M_a;
    Eigen::MatrixXf C = this->C_rb + this->C_a;
    Eigen::MatrixXf D = this->D_lin + this->D_qua;

    this->upsilon_dot = M.inverse() * (this->tau - (C * this->upsilon) 
                                       - (D * this->upsilon) - this->G_eta);

    /* Integrating Acceleration to get Velocities */

    Eigen::VectorXf upsilon_dot_sum = this->upsilon_dot + this->upsilon_dot_prev;
    this->upsilon = (upsilon_dot_sum / 2 * this->sample_time_s) + this->upsilon;

    /* Integrating Velocities to get Position */

    Eigen::VectorXf upsilon_sum = this->upsilon + this->upsilon_prev;
    this->body_pos = (upsilon_sum / 2 * this->sample_time_s) + this->body_pos;

    if (fabs(this->body_pos(3)) > pi)
    {
        this->body_pos(3) = (this->body_pos(3) / fabs(this->body_pos(3))) * (this->body_pos(3) - 2 * pi);
    }
    if (fabs(this->body_pos(4)) > pi)
    {
        this->body_pos(4) = (this->body_pos(4) / fabs(this->body_pos(4))) * (this->body_pos(4) - 2 * pi);
    }
    if (fabs(this->body_pos(5)) > pi)
    {
        this->body_pos(5) = (this->body_pos(5) / fabs(this->body_pos(5))) * (this->body_pos(5) - 2 * pi);
    }

    /* Calculate Transformation Matrix */

    this->R <<  cos(body_pos(5))*cos(body_pos(4)), -sin(body_pos(5))*cos(body_pos(3)) + cos(body_pos(5))*sin(body_pos(4))*sin(body_pos(3)),  sin(body_pos(5))*sin(body_pos(3)) + cos(body_pos(5))*cos(body_pos(3))*sin(body_pos(4)),
                sin(body_pos(5))*cos(body_pos(4)), -cos(body_pos(5))*cos(body_pos(3)) + sin(body_pos(5))*sin(body_pos(4))*sin(body_pos(3)), -cos(body_pos(5))*sin(body_pos(3)) + sin(body_pos(5))*cos(body_pos(3))*sin(body_pos(4)),
                -sin(body_pos(4)),         cos(body_pos(4))*sin(body_pos(3)), cos(body_pos(4))*cos(body_pos(3));

    this->T <<  1, sin(body_pos(3))*tan(body_pos(4)), cos(body_pos(3))*tan(body_pos(4)),
                0, cos(body_pos(3)),            -sin(body_pos(3)),
                0, sin(body_pos(3))/cos(body_pos(4)), cos(body_pos(3))/cos(body_pos(4));   
                
    this->J <<  R,      zero,
                zero,   T;

    /* Integrating Velocities to get Position on NED */

    Eigen::VectorXf eta_dot_sum = (this->J * this->upsilon) + (this->J * this->upsilon_prev);
    this->eta = (eta_dot_sum / 2 * this->sample_time_s) + this->eta;

    if (fabs(this->eta(3)) > pi)
    {
        this->eta(3) = (this->eta(3) / fabs(this->eta(3))) * (this->eta(3) - 2 * pi);
    }
    if (fabs(this->eta(4)) > pi)
    {
        this->eta(4) = (this->eta(4) / fabs(this->eta(4))) * (this->eta(4) - 2 * pi);
    }
    if (fabs(this->eta(5)) > pi)
    {
        this->eta(5) = (this->eta(5) / fabs(this->eta(5))) * (this->eta(5) - 2 * pi);
    }

    /* Transform Euler Angles to Quaternions : 3-2-1 convention */
    // double psi = this->eta(3);  // yaw
    // double body_pos(4) = 0.0;         // pitch
    // double body_pos(3) = 0.0;           // roll

    // double C_11 = cos(body_pos(4))*cos(body_pos(5));
    // double C_12 = cos(body_pos(4))*sin(body_pos(5));
    // double C_13 = -sin(body_pos(4));
    // double C_21 = sin(body_pos(3))*sin(body_pos(4))*cos(body_pos(5))-cos(body_pos(3))*sin(body_pos(5));
    // double C_22 = sin(body_pos(3))*sin(body_pos(4))*sin(body_pos(5))+cos(body_pos(3))*cos(body_pos(5));
    // double C_23 = sin(body_pos(3))*cos(body_pos(4));
    // double C_31 = cos(body_pos(3))*sin(body_pos(4))*cos(body_pos(5))+sin(body_pos(3))*sin(body_pos(5));
    // double C_32 = cos(body_pos(3))*sin(body_pos(4))*sin(body_pos(5))-sin(body_pos(3))*cos(body_pos(5));
    // double C_33 = cos(body_pos(3))*cos(body_pos(4));
    // double trace = C_11+C_22+C_33;
    
    // this->C_rot <<  C_11, C_12, C_13,
    //                 C_21, C_22, C_23,
    //                 C_31, C_32, C_33;

    // Elements are squared their value
    // this->quat << 0.25*(1+trace),
    //               0.25*(1+2*C_11-trace),
    //               0.25*(1+2*C_22-trace),
    //               0.25*(1+2*C_33-trace);
    
    // char i;
    // quat.maxCoeff(&i);

    // switch (i){
    // case 0:
    //     this->quat(0) = sqrt(this->quat(0));
    //     this->quat(1) = (C_23-C_32)/4/quat(0);
    //     this->quat(2) = (C_31-C_13)/4/quat(0);
    //     this->quat(3) = (C_12-C_21)/4/quat(0);
    //     break;
    // case 1:
    //     this->quat(1) = sqrt(this->quat(1));
    //     this->quat(0) = (C_23-C_32)/4/quat(1);
    //     this->quat(2) = (C_12+C_21)/4/quat(1);
    //     this->quat(3) = (C_31+C_13)/4/quat(1);
    //     break;
    // case 2:
    //     this->quat(2) = sqrt(this->quat(2));
    //     this->quat(0) = (C_31-C_13)/4/quat(2);
    //     this->quat(1) = (C_12+C_21)/4/quat(2);
    //     this->quat(3) = (C_23+C_32)/4/quat(2);
    //     break;
    // case 3:
    //     this->quat(3) = sqrt(this->quat(3));
    //     this->quat(0) = (C_12-C_21)/4/quat(3);
    //     this->quat(1) = (C_31+C_13)/4/quat(3);
    //     this->quat(2) = (C_23+C_32)/4/quat(3);
    //     break;
    // default:
    //     break;
    // }

    // std::cout << "Cuaterniones:" <<std::endl;
    // std::cout << this->quat[1] << std::endl;
    // std::cout << this->quat[2] << std::endl;
    // std::cout << this->quat[3] << std::endl;
    // std::cout << this->quat[0] << std::endl;
    // std::cout << "Yaw:" <<std::endl;
    // std::cout << this->eta(3) << std::endl;

    /* Update ROS Messages */

    this->linear_acceleration.x = this->upsilon_dot(0);
    this->linear_acceleration.y = this->upsilon_dot(1);
    this->linear_acceleration.z = this->upsilon_dot(2);

    this->angular_rate.x = this->upsilon(3);
    this->angular_rate.y = this->upsilon(4);
    this->angular_rate.z = this->upsilon(5);

    this->angular_position.x = this->body_pos(3);
    this->angular_position.y = this->body_pos(4);
    this->angular_position.z = this->body_pos(5); 
   
    this->velocities.linear.x = this->upsilon(0);
    this->velocities.linear.y = this->upsilon(1);
    this->velocities.linear.z = this->upsilon(2);
    this->velocities.angular.x = this->upsilon(3);
    this->velocities.angular.y = this->upsilon(4);
    this->velocities.angular.z = this->upsilon(5);
    
    this->pose.position.x = this->eta(0);
    this->pose.position.y = this->eta(1);
    this->pose.position.z = this->eta(2);
    this->pose.orientation.x = this->eta(3); //this->quat(1);
    this->pose.orientation.y = this->eta(4); //this->quat(2);
    this->pose.orientation.z = this->eta(5); //this->quat(3);
    // this->pose.orientation.w = this->quat(0);
}
