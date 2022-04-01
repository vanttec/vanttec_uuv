/** ----------------------------------------------------------------------------
 * @file: vtec_u4_6dof_dynamic_model.cpp
 * @date: March 20, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of VTec U4 6dof dynamic model.
 * -----------------------------------------------------------------------------
 **/

#include "vtec_u4_6dof_dynamic_model.hpp"

VTecU4DynamicModel::VTecU4DynamicModel(float _sample_time_s)
{
    this-> _sample_time_s = _sample_time_s;
    /* Body Parameters */
    mass            = 24;
    volume          = 0.00886;
    Ixx             = 0.900121387;
    Ixy             = -0.0001;
    Ixz             = 0.0072;
    Iyx             = -0.000186482;
    Iyy             = 1.754494427;
    Iyz             = 0.020319615;
    Izx             = 0.0072251;
    Izy             = 0.020319615;
    Izz             = 1.43389;
    weight          = 24 * 9.81;
    buoyancy        = 1000 * 9.81 * 1.005;
    x_b             = 0;
    y_b             = 0;
    z_b             = -0.2;

    /* Thruster angle parameters */
    beta            = 0.261799;
    epsilon         = 0.349066;
    rv_y            = 0.2384;
    rh_x            = 0.1867;
    rh_y            = 0.2347;
    rh_z            = 0.0175;
    b               = 0.585;
    l               = 0.382;

    /* Added Mass Parameters */
    X_u_dot         = -11.5066;
    Y_v_dot         = -8.9651;
    Z_w_dot         = -9.1344;
    K_p_dot         = -0.1851;
    M_q_dot         = -0.2810;
    N_r_dot         = -0.3475;

    /* Damping Parameters */
    X_u             = -0.3431;
    Y_v             = 0.0518;
    Z_w             = -0.5841;
    K_p             = 0.0064;
    M_q             = 0.04;
    N_r             = -0.1063;

    X_uu            = -111.7397;
    Y_vv            = -44.4058;
    Z_ww            = -157.1951;
    K_pp            = -0.4634;
    M_qq            = -0.2902;
    N_rr            = -2.2897;
}

VTecU4DynamicModel::~VTecU4DynamicModel(){}