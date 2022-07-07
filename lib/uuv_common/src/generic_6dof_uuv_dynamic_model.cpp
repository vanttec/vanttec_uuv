/** ----------------------------------------------------------------------------
 * @file: generic_6dof_uuv_dynamic_model.cpp
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

#include "generic_6dof_uuv_dynamic_model.hpp"

Generic6DOFUUVDynamicModel::Generic6DOFUUVDynamicModel(float sample_time_s)
{
    _sample_time_s = sample_time_s;

    J.resize(6,6);
    M.resize(6,6);
    M_rb.resize(6,6);
    M_a.resize(6,6);
    C.resize(6,6);
    C_rb.resize(6,6);
    C_a.resize(6,6);
    D.resize(6,6);
    D_lin.resize(6,6);
    D_qua.resize(6,6);
    g.resize(6,6);

    eta.resize(6,1);            // x, y, z, phi, theta, psi
    eta_dot.resize(6,1);
    eta_dot_prev.resize(6,1);
    nu.resize(6,1);             // u, v, w, p, q, r
    nu_dot.resize(6,1);
    nu_dot_prev.resize(6,1);
    g_eta.resize(6,1);
    tau.resize(6,1);
    u.resize(6,1);
    f.resize(6,1);

    J = Eigen::MatrixXf::Zero(6, 6);
    R = Eigen::Matrix3f::Zero(3,3);
    T = Eigen::Matrix3f::Zero(3,3);

    M = Eigen::MatrixXf::Zero(6, 6);
    M_rb = Eigen::MatrixXf::Zero(6, 6);
    M_a = Eigen::MatrixXf::Zero(6, 6);
    C = Eigen::MatrixXf::Zero(6, 6);
    C_rb = Eigen::MatrixXf::Zero(6, 6);
    C_a = Eigen::MatrixXf::Zero(6, 6);
    D = Eigen::MatrixXf::Zero(6, 6);
    D_lin = Eigen::MatrixXf::Zero(6, 6);
    D_qua = Eigen::MatrixXf::Zero(6, 6);
    g_eta = Eigen::MatrixXf::Zero(6, 1);

    eta << 0,
           0,
           0,
           0,
           0,
           0;
     
    eta_dot << 0,
               0,
               0,
               0,
               0,
               0;

    eta_dot_prev << 0,
                    0,
                    0,
                    0,
                    0,
                    0;
    
    nu << 0,
          0,
          0,
          0,
          0,
          0;
    
    nu_dot << 0,
              0,
              0,
              0,
              0,
              0;
    
    nu_dot_prev << 0,
                   0,
                   0,
                   0,
                   0,
                   0;

    f << 0,
         0,
         0,
         0,
         0,
         0;

    g = Eigen::MatrixXf::Zero(6, 6);

    tau << 0,
           0,
           0,
           0,
           0,
           0;

    u << 0,
           0,
           0,
           0,
           0,
           0;

    MAX_FORCE_X = 0.0;
    MAX_FORCE_Y = 0.0;
    MAX_FORCE_Z = 0.0;
    MAX_TORQUE_K = 0.0;
    MAX_TORQUE_M = 0.0;
    MAX_TORQUE_N = 0.0;

    eta_pose.x = 0;
    eta_pose.y = 0;
    eta_pose.z = 0;
    eta_pose.phi = 0;
    eta_pose.theta = 0;
    eta_pose.psi = 0;

    velocities.linear.x = 0;
    velocities.linear.y = 0;
    velocities.linear.z = 0;
    velocities.angular.x = 0;
    velocities.angular.y = 0;
    velocities.angular.z = 0;

    accelerations.linear.x = 0;
    accelerations.linear.y = 0;
    accelerations.linear.z = 0;
    accelerations.angular.x = 0;
    accelerations.angular.y = 0;
    accelerations.angular.z = 0;
}

Generic6DOFUUVDynamicModel::~Generic6DOFUUVDynamicModel(){}

void Generic6DOFUUVDynamicModel::CalculateTransformation()
{
    R <<    std::cos(eta(5))*std::cos(eta(4)),      -std::sin(eta(5))*std::cos(eta(3)) + std::cos(eta(5))*std::sin(eta(4))*std::sin(eta(3)),     std::sin(eta(5))*std::sin(eta(3)) + std::cos(eta(5))*std::cos(eta(3))*std::sin(eta(4)),
            std::sin(eta(5))*std::cos(eta(4)),       std::cos(eta(5))*std::cos(eta(3)) + std::sin(eta(3))*std::sin(eta(4))*std::sin(eta(5)),    -std::cos(eta(5))*std::sin(eta(3)) + std::sin(eta(4))*std::sin(eta(5))*std::cos(eta(3)),
           -std::sin(eta(4)),                        std::cos(eta(4))*std::sin(eta(3)),                                                          std::cos(eta(4))*std::cos(eta(3));

    T <<    1,     std::sin(eta(3))*std::tan(eta(4)),  std::cos(eta(3))*std::tan(eta(4)),
            0,     std::cos(eta(3)),                  -std::sin(eta(3)),
            0,     std::sin(eta(3))/std::cos(eta(4)),  std::cos(eta(3))/std::cos(eta(4));

    J << R,                              Eigen::Matrix3f::Zero(3, 3),
         Eigen::Matrix3f::Zero(3, 3),    T;
}

void Generic6DOFUUVDynamicModel::CalculateCoriolis()
{
    /* Rigid Body Coriolis Matrix */

    float m_u = m  * nu(0);
    float m_v = m  * nu(1);
    float m_w = m  * nu(2);
    float ixx_p = Ixx * nu(3);
    float iyy_q = Iyy * nu(4);
    float izz_r = Izz * nu(5);
    
    C_rb << 0,   0,   0,   0,    m_w,    -m_v,
            0,   0,   0,   -m_w,  0,     m_u,
            0,   0,   0,   m_v,   -m_u,   0,
            0,   m_w,  -m_v, 0,    -izz_r, iyy_q,
            -m_w, 0 ,  m_u,  izz_r, 0,     -ixx_p,
            m_v,  -m_u, 0,   -iyy_q, ixx_p,  0;

    /* Hydrodynamic Added Mass Coriolis Matrix */

    float a1 = X_u_dot * nu(0);
    float a2 = Y_v_dot * nu(1);
    float a3 = Z_w_dot * nu(2);
    float a4 = K_p_dot * nu(3);
    float a5 = M_q_dot * nu(4);
    float a6 = N_r_dot * nu(5);
    
    C_a << 0,     0,    0,    0,  -a3,  a2,
           0,     0,    0,   a3,   0,  -a1,
           0,     0,    0,  -a2,  a1,   0,
           0,   -a3,  a2,   0,  -a6,  a5,
           a3,   0,  -a1,  a6,   0,  -a4,
           -a2,  a1,   0,  -a5,   a4,  0;
    
    C = C_rb + C_a;
    // std::cout << "C:" << C << std::endl;

}

void Generic6DOFUUVDynamicModel::CalculateDamping()
{    
    /* Hydrodynamic Damping */

    D_lin << -(X_u), 0,      0,    0,   0,    0,
                0,   -(Y_v), 0,    0,   0,    0,
                0,   0,    -(Z_w), 0,   0,    0,
                0,   0,      0, -K_p,   0,    0,
                0,   0,      0,    0, -M_q,   0,
                0,   0,      0,    0,   0, -(N_r);

    D_qua << -(X_uu * fabs(nu(0))), 0, 0, 0, 0, 0,
             0, -(Y_vv * fabs(nu(1))), 0, 0, 0, 0,
             0, 0, -(Z_ww * fabs(nu(2))), 0, 0, 0,
             0, 0, 0, -(K_pp * fabs(nu(3))), 0, 0,
             0, 0, 0, 0, -(M_qq * fabs(nu(4))), 0,
             0, 0, 0, 0, 0, -(N_rr * fabs(nu(5)));
    
    D = D_lin + D_qua;
    // std::cout << "D:" << D << std::endl;

}

void Generic6DOFUUVDynamicModel::ThrustCallback(const vanttec_uuv::ThrustControl& _thrust)
{
    tau << _thrust.tau_x,
           _thrust.tau_y,
           _thrust.tau_z,
           _thrust.tau_phi,
           _thrust.tau_theta,
           _thrust.tau_psi;
}

void Generic6DOFUUVDynamicModel::CalculateStates()
{
    CalculateTransformation();
    nu_dot_prev = nu_dot;
    eta_dot_prev = J * nu;

    /* Rigid Body Mass Matrix */

    M_rb << m, 0, 0, 0, 0, 0,
            0, m, 0, 0, 0, 0,
            0, 0, m, 0, 0, 0,
            0, 0, 0, Ixx, 0, 0,
            0, 0, 0, 0, Iyy, 0,
            0, 0, 0, 0, 0, Izz;

    /* Hydrodynamic Added Mass Matrix */

    M_a << X_u_dot, 0, 0, 0, 0, 0,
           0, Y_v_dot, 0, 0, 0, 0,
           0, 0, Z_w_dot, 0, 0, 0,
           0, 0, 0, K_p_dot, 0, 0,
           0, 0, 0, 0, M_q_dot, 0,
           0, 0, 0, 0, 0, N_r_dot;

    M = M_rb + M_a;
    // std::cout << "M:" << M << std::endl;
    CalculateCoriolis();

    CalculateDamping();

    /* Restoring Forces */
    
    g_eta <<  (W - B)*sin(eta(4)),
              -(W - B)*cos(eta(4))*sin(eta(3)),
              -(W - B)*cos(eta(4))*cos(eta(3)),
              rb_y*B*cos(eta(4))*cos(eta(4)) - rb_z*B*cos(eta(4))*sin(eta(4)),
              -rb_z*B*sin(eta(4)) - rb_x*B*cos(eta(4))*cos(eta(4)),
              rb_x*cos(eta(4))*sin(eta(4)) + rb_y*B*sin(eta(4));

    /* 6 DoF State Calculation */

    g = M.inverse();
    f = -M.inverse() * ((C * nu) + (D * nu) + g_eta);
    // std::cout << "f:" << f << std::endl;
    // std::cout << "g:" << g << std::endl;

    u = tau;
    nu_dot = f + g*u;
    // nu_dot =  M.inverse() * (tau - (C * nu) - (D * nu) - g_eta);

    /* Integrating Acceleration to get Velocities */

    Eigen::VectorXf nu_dot_sum = nu_dot + nu_dot_prev;
    nu = (nu_dot_sum / 2 * _sample_time_s) + nu;

    /* Integrating Velocities to get Position */

    eta_dot = J * nu;

    Eigen::VectorXf eta_dot_sum = eta_dot + eta_dot_prev;
    eta = (eta_dot_sum / 2 * _sample_time_s) + eta;

    if (fabs(eta(3)) > M_PI)
    {
        eta(3) = (eta(3) / fabs(eta(3))) * (fabs(eta(3)) - 2 * M_PI);
    }
    if (fabs(eta(4)) > M_PI)
    {
        eta(4) = (eta(4) / fabs(eta(4))) * (fabs(eta(4)) - 2 * M_PI);
    }
    if (fabs(eta(5)) > M_PI)
    {
        eta(5) = (eta(5) / fabs(eta(5))) * (fabs(eta(5)) - 2 * M_PI);
    }

    /* Transform Euler Angles to Quaternions : 3-2-1 convention */
    // double psi = eta(3);  // yaw
    // double eta(4) = 0.0;         // pitch
    // double eta(4) = 0.0;           // roll

    // double C_11 = cos(eta(4))*cos(Theta(2));
    // double C_12 = cos(eta(4))*sin(Theta(2));
    // double C_13 = -sin(eta(4));
    // double C_21 = sin(eta(4))*sin(eta(4))*cos(Theta(2))-cos(eta(4))*sin(Theta(2));
    // double C_22 = sin(eta(4))*sin(eta(4))*sin(Theta(2))+cos(eta(4))*cos(Theta(2));
    // double C_23 = sin(eta(4))*cos(eta(4));
    // double C_31 = cos(eta(4))*sin(eta(4))*cos(Theta(2))+sin(eta(4))*sin(Theta(2));
    // double C_32 = cos(eta(4))*sin(eta(4))*sin(Theta(2))-sin(eta(4))*cos(Theta(2));
    // double C_33 = cos(eta(4))*cos(eta(4));
    // double trace = C_11+C_22+C_33;
    
    // C_rot <<  C_11, C_12, C_13,
    //                 C_21, C_22, C_23,
    //                 C_31, C_32, C_33;

    // Elements are squared their value
    // quat << 0.25*(1+trace),
    //               0.25*(1+2*C_11-trace),
    //               0.25*(1+2*C_22-trace),
    //               0.25*(1+2*C_33-trace);
    
    // char i;
    // quat.maxCoeff(&i);

    // switch (i){
    // case 0:
    //     quat(0) = sqrt(quat(0));
    //     quat(1) = (C_23-C_32)/4/quat(0);
    //     quat(2) = (C_31-C_13)/4/quat(0);
    //     quat(3) = (C_12-C_21)/4/quat(0);
    //     break;
    // case 1:
    //     quat(1) = sqrt(quat(1));
    //     quat(0) = (C_23-C_32)/4/quat(1);
    //     quat(2) = (C_12+C_21)/4/quat(1);
    //     quat(3) = (C_31+C_13)/4/quat(1);
    //     break;
    // case 2:
    //     quat(2) = sqrt(quat(2));
    //     quat(0) = (C_31-C_13)/4/quat(2);
    //     quat(1) = (C_12+C_21)/4/quat(2);
    //     quat(3) = (C_23+C_32)/4/quat(2);
    //     break;
    // case 3:
    //     quat(3) = sqrt(quat(3));
    //     quat(0) = (C_12-C_21)/4/quat(3);
    //     quat(1) = (C_31+C_13)/4/quat(3);
    //     quat(2) = (C_23+C_32)/4/quat(3);
    //     break;
    // default:
    //     break;
    // }

    // std::cout << "Cuaterniones:" <<std::endl;
    // std::cout << quat[1] << std::endl;
    // std::cout << quat[2] << std::endl;
    // std::cout << quat[3] << std::endl;
    // std::cout << quat[0] << std::endl;
    // std::cout << "Yaw:" <<std::endl;
    // std::cout << eta(3) << std::endl;

    /* Update ROS Messages */

    accelerations.linear.x = nu_dot(0);
    accelerations.linear.y = nu_dot(1);
    accelerations.linear.z = nu_dot(2);
    accelerations.angular.x = nu_dot(0);
    accelerations.angular.y = nu_dot(1);
    accelerations.angular.z = nu_dot(2);

    velocities.linear.x = nu(0);
    velocities.linear.y = nu(1);
    velocities.linear.z = nu(2);
    velocities.angular.x = nu(3);
    velocities.angular.y = nu(4);
    velocities.angular.z = nu(5);
    
    eta_pose.x = eta(0);
    eta_pose.y = eta(1);
    eta_pose.z = eta(2);
    eta_pose.phi = eta(3);
    eta_pose.theta = eta(4);
    eta_pose.psi = eta(5);
}