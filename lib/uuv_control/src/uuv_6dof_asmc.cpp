/** ----------------------------------------------------------------------------
 * @file: uuv_6dof_controller.cpp
 * @date: March 2, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 6-DOF adaptive sliding mode controller class
 * -----------------------------------------------------------------------------
 * */

#include "uuv_6dof_asmc.hpp"

UUV_6DOF_ASMC::UUV_6DOF_ASMC(float _sample_time_s, const float _kpid_u[3], const float _kpid_v[3], const float _kpid_z[3], const float _kpid_psi[3]){
                                    : x_controller(_sample_time_s, _kpid_u, LINEAR_DOF)
                                    , y_controller(_sample_time_s, _kpid_v, LINEAR_DOF)
                                    , z_controller(_sample_time_s, _kpid_z, LINEAR_DOF)
                                    , phi_controller(_sample_time_s, _kpid_psi, ANGULAR_DOF)
                                    , theta_controller(_sample_time_s, _kpid_psi, ANGULAR_DOF)
                                    , psi_controller(_sample_time_s, _kpid_psi, ANGULAR_DOF)
{
    this->g_x << (1 / (mass - X_u_dot)),
                 (1 / (mass - Y_v_dot)),
                 (1 / (mass - Z_w_dot)),
                 (1 / (Izz - N_r_dot));

    this->surge_speed_controller.g_x    = g_x(0);
    this->sway_speed_controller.g_x     = g_x(1);
    this->depth_controller.g_x          = g_x(2);
    this->heading_controller.g_x        = g_x(3);

    this->f_x << 0,
                 0,
                 0,
                 0;
}