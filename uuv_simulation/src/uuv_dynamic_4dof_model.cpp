#include "uuv_dynamic_4dof_model.hpp"

#include <math.h>
#include <stdio.h>

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

void UUVDynamic4DOFModel::ThrustCallback(const uuv_control::ThrustControl& _thrust)
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

    std::cout << this->body_pos << std::endl;

    /* Update ROS Messages */

    this->linear_acceleration.x = upsilon_dot(0);
    this->linear_acceleration.y = upsilon_dot(1);
    this->linear_acceleration.z = upsilon_dot(2);

    this->angular_rate.x = 0;
    this->angular_rate.y = 0;
    this->angular_rate.z = upsilon(3);

    this->angular_position.x = 0;
    this->angular_position.y = 0;
    this->angular_position.z = body_pos(3); 

    this->velocities.linear.x = upsilon(0);
    this->velocities.linear.y = upsilon(1);
    this->velocities.linear.z = upsilon(2);
    this->velocities.angular.z = upsilon(3);

    this->pose.position.x = body_pos(0);
    this->pose.position.y = body_pos(1);
    this->pose.position.z = body_pos(2);
    this->pose.orientation.z = body_pos(3);
}
