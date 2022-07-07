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

VTecU4DynamicModel::VTecU4DynamicModel(float _sample_time_s) : Generic6DOFUUVDynamicModel(_sample_time_s)
{
    /* Physical Parameters */

    m     = 24;
    W   = 235.44;
    volume   = 0.0252;
    B = 247.212;
    Ixx      = 0.900121387;
    Ixy      = 0;
    Ixz      = 0;
    Iyx      = 0;
    Iyy      = 1.754494427;
    Iyz      = 0;
    Izx      = 0;
    Izy      = 0;
    Izz      = 1.43389;

    /* Added Mass Parameters */

    X_u_dot  = 16.8374;
    Y_v_dot  = 20.2748;
    Z_w_dot  = 35.3180;
    K_p_dot  = 0.2165;
    M_q_dot  = 0.6869;
    N_r_dot  = 0.6157;

    /* Damping Parameters */

    X_u      = -0.3431;
    Y_v      = 0.0518;
    Z_w      = -0.5841;
    K_p      = 0.0064;
    M_q      = 0.04;
    N_r      = -0.1063;
    X_uu     = -111.7397;
    Y_vv     = -44.4058;
    Z_ww     = -157.1951;
    K_pp     = -0.4634;
    M_qq     = -0.2902;
    N_rr     = -2.2897;

    /* Distance from origin to center of buoyancy  */

    rb_x = 0;
    rb_y = 0;
    rb_z = -0.10726;

    /* Thruster configuration parameters */

    beta            = 0.349066;
    var_epsilon     = 0.261799;
    alpha           = 1.325054;
    delta           = 0.43301619;
    gamma           = 1.8326;
    rh_x            = 0.1867;
    rh_y            = 0.2347;
    rh_z            = 0.0175;
    rv_x            = 0;
    rv_y            = 0.2384;
    rv_z            = 0;

    /* Maximum forces and torques per degree of freedom */
    MAX_FORCE_X = 127; 
    MAX_FORCE_Y = 34;  
    MAX_FORCE_Z = 118; 
    MAX_TORQUE_K = 28;  
    MAX_TORQUE_M = 9.6; 
    MAX_TORQUE_N = 36.6;
}

VTecU4DynamicModel::~VTecU4DynamicModel(){}

void VTecU4DynamicModel::UpdateThrust()
{
    float c_delta = fabs(std::cos(delta));
    float c_alpha = fabs(std::cos(alpha));
    float c_gamma = fabs(std::cos(gamma));
    
    L <<  c_delta,   c_delta,    -c_delta,   -c_delta,    0,     0,
         -c_alpha,   c_alpha,    -c_alpha,    c_alpha,    0,     0,
         -c_gamma,  -c_gamma,     c_gamma,    c_gamma,   -1,    -1;
         -(rh_y*c_gamma + rh_z*c_alpha),    rh_y*c_gamma + rh_z*c_alpha,    rh_y*c_gamma - rh_z*c_alpha,    -(rh_y*c_gamma - rh_z*c_alpha), -rv_y, rv_y,
         rh_x*c_gamma - rh_z*c_delta,       rh_x*c_gamma - rh_z*c_delta,    rh_x*c_gamma + rh_z*c_delta,     rh_x*c_gamma + rh_z*c_delta,    0,   0,
         -(rh_x*c_alpha + rh_y*c_delta),    rh_x*c_alpha + rh_y*c_delta,    rh_x*c_alpha + rh_y*c_delta,    -(rh_x*c_alpha + rh_y*c_delta),   0,   0;

    Thrust << (L.completeOrthogonalDecomposition().pseudoInverse())*tau;
}