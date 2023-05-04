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
                     0;
    
    this->upsilon_prev << 0,
                          0,
                          0,
                          0;
    
    this->upsilon_dot << 0,
                         0,
                         0,
                         0;
    
    this->upsilon_dot_prev << 0,
                              0,
                              0,
                              0;

    this->body_pos << 0,
                      0,
                      0,
                      0;

    this->tau << 0,
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

}

UUVDynamic4DOFModel::~UUVDynamic4DOFModel(){}

void UUVDynamic4DOFModel::ThrustCallback(const vanttec_uuv::ThrustControl& _thrust)
{
    this->tau << _thrust.tau_x,
                 _thrust.tau_y,
                 _thrust.tau_z,
                 _thrust.tau_yaw;
}

void UUVDynamic4DOFModel::CalculateStates()
{
    this->upsilon_dot_prev = this->upsilon_dot;
    this->upsilon_prev = this->upsilon;

    /* Rigid Body Mass Matrix */

    this->M_rb << mass, 0, 0, 0,
                  0, mass, 0, 0,
                  0, 0, mass, 0,
                  0, 0, 0, Izz;

    /* Hydrodynamic Added Mass Matrix */

    this->M_a << X_u_dot, 0, 0, 0,
                 0, Y_v_dot, 0, 0,
                 0, 0, Z_w_dot, 0,
                 0, 0, 0, N_r_dot;

    /* Rigid Body Coriolis Matrix */

    float rb_a_1 = mass * this->upsilon(0);
    float rb_a_2 = mass * this->upsilon(1);
    
    this->C_rb << 0, 0, 0, -rb_a_2,
                  0, 0, 0, rb_a_1,
                  0, 0, 0, 0,
                  rb_a_2, -rb_a_1, 0, 0;

    /* Hydrodynamic Added Mass Coriolis Matrix */

    float a_a_1 = X_u_dot * this->upsilon(0);
    float a_a_2 = Y_v_dot * this->upsilon(1);
    
    this->C_a << 0, 0, 0, a_a_2,
                 0, 0, 0, -a_a_1,
                 0, 0, 0, 0,
                 -a_a_2, a_a_1, 0, 0;
    
    /* Hydrodynamic Damping */

    this->D_lin << -(X_u), 0, 0, 0,
                   0, -(Y_v), 0, 0,
                   0, 0, -(Z_w), 0,
                   0, 0, 0, -(N_r);

    this->D_qua << -(X_uu * fabs(this->upsilon(0))), 0, 0, 0,
                   0, -(Y_vv * fabs(this->upsilon(1))), 0, 0,
                   0, 0, -(Z_ww * fabs(this->upsilon(2))), 0,
                   0, 0, 0, -(N_rr * fabs(this->upsilon(3)));

    /* Restoring Forces */
    
    this->G_eta << (weight - buoyancy) * sin(theta_b),
                   -(weight - buoyancy) * cos(theta_b) * sin(phi_b),
                   -(weight - buoyancy) * cos(theta_b) * cos(phi_b),
                   0;

    /* 4 DoF State Calculation */

    Eigen::Matrix4f M = this->M_rb - this->M_a;
    Eigen::Matrix4f C = this->C_rb + this->C_a;
    Eigen::Matrix4f D = this->D_lin + this->D_qua;

    this->upsilon_dot = M.inverse() * (this->tau - (C * this->upsilon) 
                                       - (D * this->upsilon) - this->G_eta);

    /* Integrating Acceleration to get Velocities */

    Eigen::Vector4f upsilon_dot_sum = this->upsilon_dot + this->upsilon_dot_prev;
    this->upsilon = (upsilon_dot_sum / 2 * this->sample_time_s) + this->upsilon;

    /* Integrating Velocities to get Position */

    Eigen::Vector4f upsilon_sum = this->upsilon + this->upsilon_prev;
    this->body_pos = (upsilon_sum / 2 * this->sample_time_s) + this->body_pos;

    if (fabs(this->body_pos(3)) > pi)
    {
        this->body_pos(3) = (this->body_pos(3) / fabs(this->body_pos(3))) * (this->body_pos(3) - 2 * pi);
    }

    /* Calculate Transformation Matrix */

    this->J << cos(this->body_pos(3)), -sin(this->body_pos(3)), 0, 0,
               sin(this->body_pos(3)), cos(this->body_pos(3)), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    /* Integrating Velocities to get Position on NED */

    Eigen::Vector4f eta_dot_sum = (this->J * this->upsilon) + (this->J * this->upsilon_prev);
    this->eta = (eta_dot_sum / 2 * this->sample_time_s) + this->eta;

    if (fabs(this->eta(3)) > pi)
    {
        this->eta(3) = (this->eta(3) / fabs(this->eta(3))) * (this->eta(3) - 2 * pi);
    }

    /* Transform Euler Angles to Quaternions : 3-2-1 convention */
    // double psi = this->eta(3);  // yaw
    // double theta = 0.0;         // pitch
    // double phi = 0.0;           // roll

    // double C_11 = cos(theta)*cos(psi);
    // double C_12 = cos(theta)*sin(psi);
    // double C_13 = -sin(theta);
    // double C_21 = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    // double C_22 = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    // double C_23 = sin(phi)*cos(theta);
    // double C_31 = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    // double C_32 = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    // double C_33 = cos(phi)*cos(theta);
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

    this->angular_rate.x = 0;
    this->angular_rate.y = 0;
    this->angular_rate.z = this->upsilon(3);

    this->angular_position.x = 0;
    this->angular_position.y = 0;
    this->angular_position.z = this->body_pos(3); 
   
    this->velocities.linear.x = this->upsilon(0);
    this->velocities.linear.y = this->upsilon(1);
    this->velocities.linear.z = this->upsilon(2);
    this->velocities.angular.z = this->upsilon(3);
    
    this->pose.position.x = this->eta(0);
    this->pose.position.y = this->eta(1);
    this->pose.position.z = this->eta(2);
    // this->pose.orientation.x = this->quat(1);
    // this->pose.orientation.y = this->quat(2);
    this->pose.orientation.z = this->eta(3); //this->quat(3);
    // this->pose.orientation.w = this->quat(0);
}
