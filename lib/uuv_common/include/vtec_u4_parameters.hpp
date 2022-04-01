/** ----------------------------------------------------------------------------
 * @file: vtec_u4__parameters.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Mathematical constants that describe the UUV model for simulation.
 * -----------------------------------------------------------------------------
 * */

#ifndef __VTEC_U4__PARAMETERS_H__
#define __VTEC_U4__PARAMETERS_H__

/* Constants */
        
static const float rho             = 1000;
static const float g               = 9.81;
static const float pi              = 3.14159;

/* Body Parameters */

static const float mass            = 24;
static const float volume          = 0.00886;
static const float Ixx             = 0.900121387;
static const float Ixy             = -0.0001;
static const float Ixz             = 0.0072;
static const float Iyx             = -0.000186482;
static const float Iyy             = 1.754494427;
static const float Iyz             = 0.020319615;
static const float Izx             = 0.0072251;
static const float Izy             = 0.020319615;
static const float Izz             = 1.43389;
static const float weight          = 24 * 9.81;
static const float buoyancy        = 1000 * 9.81 * 1.005;
static const float x_b             = 0;
static const float y_b             = 0;
static const float z_b             = -0.2;

/* Thruster angle parameters */
static const float beta            = 0.261799;
static const float epsilon         = 0.349066;
static const float rv_y            = 0.2384;
static const float rh_x            = 0.1867;
static const float rh_y            = 0.2347;
static const float rh_z            = 0.0175;
static const float b               = 0.585;
static const float l               = 0.382;

/* Added Mass Parameters */

static const float X_u_dot         = -11.5066;
static const float Y_v_dot         = -8.9651;
static const float Z_w_dot         = -9.1344;
static const float K_p_dot         = -0.1851;
static const float M_q_dot         = -0.2810;
static const float N_r_dot         = -0.3475;

/* Damping Parameters */

static const float X_u             = -0.3431;
static const float Y_v             = 0.0518;
static const float Z_w             = -0.5841;
static const float K_p             = 0.0064;
static const float M_q             = 0.04;
static const float N_r             = -0.1063;

static const float X_uu            = -111.7397;
static const float Y_vv            = -44.4058;
static const float Z_ww            = -157.1951;
static const float K_pp            = -0.4634;
static const float M_qq            = -0.2902;
static const float N_rr            = -2.2897;

/* Hardcoded Angles for Roll and Pitch */

static const float theta_b         = 0;
static const float phi_b           = 0;

/* Max Thrust Values for different DoFs */

static const float MAX_THRUST_SURGE = 100;
static const float MAX_THRUST_SWAY  = 100;
static const float MAX_THRUST_HEAVE = 100;
static const float MAX_THRUST_YAW   = 100;

/* Controller Tuned Constants */

static const float Kpid_u[3]       = {7.5, 0.025, 0.4};
static const float Kpid_v[3]       = {7.5, 0.025, 0.4};
static const float Kpid_z[3]       = {1.1, 0, 1.5};
static const float Kpid_psi[3]     = {1.0, 0, 1.75};

#endif
