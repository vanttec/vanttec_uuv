/** ----------------------------------------------------------------------------
 * @file: vtec_u3_gamma_parameters.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Mathematical constants that describe the UUV model for simulation.
 * -----------------------------------------------------------------------------
 * */

#ifndef __VTEC_U3_GAMMA_PARAMETERS_H__
#define __VTEC_U3_GAMMA_PARAMETERS_H__

/* Constants */
        
static const float rho             = 1000;
static const float g               = 9.81;
static const float pi              = 3.14159;

/* Body Parameters */

static const float mass            = 13.37;
static const float volume          = 0.00886;
static const float Ixx             = 0.4977;
static const float Ixy             = 0.0027;
static const float Ixz             = -0.0574;
static const float Iyx             = 0.0027;
static const float Iyy             = 0.3709;
static const float Iyz             = -0.0037;
static const float Izx             = -0.0574;
static const float Izy             = -0.0037;
static const float Izz             = 0.6488;
static const float thruster_theta  = 3.14159 / 2;
static const float b               = 0.585;
static const float l               = 0.382;
static const float weight          = 13.37 * 9.81;
static const float buoyancy        = 1000 * 9.81 * 0.00886;

/* Added Mass Parameters */

static const float X_u_dot         = -11.5066;
static const float Y_v_dot         = -8.9651;
static const float Z_w_dot         = -9.1344;
static const float K_p_dot         = -0.1851;
static const float M_q_dot         = -0.2810;
static const float N_r_dot         = -0.3475;

/* Damping Parameters */

static const float X_u             = 0.6969;
static const float Y_v             = -0.044;
static const float Z_w             = 2.5418;
static const float K_p             = -0.0521;
static const float M_q             = -0.0431;
static const float N_r             = -0.1124;

static const float X_uu            = -45.808;
static const float Y_vv            = -41.282;
static const float Z_ww            = -42.243;
static const float K_pp            = -0.3185;
static const float M_qq            = -0.4752;
static const float N_rr            = -0.607;

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