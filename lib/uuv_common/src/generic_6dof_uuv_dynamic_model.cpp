/** ----------------------------------------------------------------------------
 * @file: generic_6dof_uuv_dynamic_model.cpp
 * @date: March 20, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of a generic 6dof UUV model.
 * -----------------------------------------------------------------------------
 **/

#include "generic_6dof_uuv_dynamic_model.hpp"

Generic6DOFUUVDynamicModel::Generic6DOFUUVDynamicModel(float sample_time_s)
{
    this->sample_time_s = sample_time_s;

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

    this->Theta << 0,
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

    this->attitude.x = 0;
    this->attitude.y = 0;
    this->attitude.z = 0;

    this->velocities.linear.x = 0;
    this->velocities.linear.y = 0;
    this->velocities.linear.z = 0;
    
    this->velocities.angular.x = 0;
    this->velocities.angular.y = 0;
    this->velocities.angular.z = 0;
}

Generic6DOFUUVDynamic6DOF::~Generic6DOFUUVDynamic6DOF(){}

void Generic6DOFUUVDynamic6DOF::ThrustCallback(const vanttec_uuv::ThrustControl& _thrust)
{
    this->tau << _thrust.tau_x,
                 _thrust.tau_y,
                 _thrust.tau_z,
                 _thrust.tau_phi,
                 _thrust.tau_theta,
                 _thrust.tau_yaw;
}

void Generic6DOFUUVDynamic6DOF::CalculateStates()
{
    this->J = uuv_common::CalculateTransformation(phi, theta, psi);

    this->upsilon_dot_prev = this->upsilon_dot;
    // this->upsilon_prev = this->upsilon;
    eta_dot_prev = this->J * this->upsilon; //this->J * this->upsilon_prev; 

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

    float m_u = mass  * this->upsilon(0);
    float m_v = mass  * this->upsilon(1);
    float m_w = mass  * this->upsilon(2);
    float ixx_p = Ixx * this->upsilon(3);
    float iyy_q = Iyy * this->upsilon(4);
    float izz_r = Izz * this->upsilon(5);
    
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
    
    this->G_eta <<  (weight - buoyancy)*sin(Theta(1)),
                   -(weight - buoyancy)*cos(Theta(1))*sin(Theta(1)),
                   -(weight - buoyancy)*cos(Theta(1))*cos(Theta(1)),
                   y_b*buoyancy*cos(Theta(1))*cos(Theta(1)) - z_b*buoyancy*cos(Theta(1))*sin(Theta(1)),
                   -z_b*buoyancy*sin(Theta(1)) - x_b*buoyancy*cos(Theta(1))*cos(Theta(1)),
                   x_b*cos(Theta(1))*sin(Theta(1)) + y_b*buoyancy*sin(Theta(1));

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

    eta_dot = this->J * this->upsilon;

    Eigen::VectorXf eta_dot_sum = eta_dot + eta_dot_prev;
    this->eta = (eta_dot_sum / 2 * this->sample_time_s) + this->eta;

    if (fabs(this->eta(3)) > pi)
    {
        this->eta(3) = (this->eta(3) / fabs(this->eta(3))) * (fabs(this->eta(3)) - 2 * pi);
    }
    if (fabs(this->eta(4)) > pi)
    {
        this->eta(4) = (this->eta(4) / fabs(this->eta(4))) * (fabs(this->eta(4)) - 2 * pi);
    }
    if (fabs(this->eta(5)) > pi)
    {
        this->eta(5) = (this->eta(5) / fabs(this->eta(5))) * (fabs(this->eta(5)) - 2 * pi);
    }

    /* Transform Euler Angles to Quaternions : 3-2-1 convention */
    // double psi = this->eta(3);  // yaw
    // double Theta(1) = 0.0;         // pitch
    // double Theta(1) = 0.0;           // roll

    // double C_11 = cos(Theta(1))*cos(Theta(2));
    // double C_12 = cos(Theta(1))*sin(Theta(2));
    // double C_13 = -sin(Theta(1));
    // double C_21 = sin(Theta(1))*sin(Theta(1))*cos(Theta(2))-cos(Theta(1))*sin(Theta(2));
    // double C_22 = sin(Theta(1))*sin(Theta(1))*sin(Theta(2))+cos(Theta(1))*cos(Theta(2));
    // double C_23 = sin(Theta(1))*cos(Theta(1));
    // double C_31 = cos(Theta(1))*sin(Theta(1))*cos(Theta(2))+sin(Theta(1))*sin(Theta(2));
    // double C_32 = cos(Theta(1))*sin(Theta(1))*sin(Theta(2))-sin(Theta(1))*cos(Theta(2));
    // double C_33 = cos(Theta(1))*cos(Theta(1));
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

    this->velocities.linear.x = this->upsilon(0);
    this->velocities.linear.y = this->upsilon(1);
    this->velocities.linear.z = this->upsilon(2);
    this->velocities.angular.x = this->upsilon(3);
    this->velocities.angular.y = this->upsilon(4);
    this->velocities.angular.z = this->upsilon(5);
    
    this->eta_pose.x = this->eta(0);
    this->eta_pose.y = this->eta(1);
    this->eta_pose.z = this->eta(2);
    this->eta_pose.phi = this->eta(3);
    this->eta_pose.theta = this->eta(4);
    this->eta_pose.psi = this->eta(5);
}