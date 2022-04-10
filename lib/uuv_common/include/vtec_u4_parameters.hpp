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

/* Physical Parameters */

static const float m               = 24;            // Mass
static const float W               = 235.44;        // Weight
static const float volume          = 0.0252;
static const float B               = 247.212;       // Buoyancy
static const float Ixx             = 0.900121387;
static const float Ixy             = 0;
static const float Ixz             = 0;
static const float Iyx             = 0;
static const float Iyy             = 1.754494427;
static const float Iyz             = 0;
static const float Izx             = 0;
static const float Izy             = 0;
static const float Izz             = 1.43389;

/* Added Mass Parameters */

static const float X_u_dot         = 16.8374;
static const float Y_v_dot         = 20.2748;
static const float Z_w_dot         = 35.3180;
static const float K_p_dot         = 0.2165;
static const float M_q_dot         = 0.6869;
static const float N_r_dot         = 0.6157;

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

/* Distance from origin to center of mass */
// They are the same

/* Distance from origin to center of buoyancy  */

static const float rb_x            = 0;
static const float rb_y            = 0;
static const float rb_z            = -0.10726;

/* Thruster configuration parameters */

static const float beta            = 0.349066;
static const float var_epsilon     = 0.261799;
static const float alpha           = 1.325054;
static const float delta           = 0.43301619;
static const float gamma           = 1.8326;
static const float rh_x            = 0.1867;
static const float rh_y            = 0.2347;
static const float rh_z            = 0.0175;
static const float rv_x            = 0;
static const float rv_y            = 0.2384;
static const float rv_z            = 0;

/* BlueRobotics Thrust Limits */

static const float MAX_THRUST = 35;
static const float MIN_THRUST = -35;

#endif
