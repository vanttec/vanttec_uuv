/** ----------------------------------------------------------------------------
 * @file: asmc_guidance.cpp
 * @date: June 20, 2021
 * @author: Carlos Medina
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ASMC Waypoint Guidance Class
 * -----------------------------------------------------------------------------
 * */

#include "asmc_guidance.hpp"

ASMC_GUIDANCE::ASMC_GUIDANCE(double _sample_time_s, const double _Ka,  const double _K2, const double _Kalpha, const double _Kmin, const double _miu, const DOFControllerType_E _type)
                            :ASMC(_sample_time_s,_K2,_Kalpha,_Kmin,_miu,_type)
{
    Ka = _Ka;
    error_i = 0;
    U = 0;
    Uax = 0;
    desired_dot_error = 0;
}

ASMC_GUIDANCE::~ASMC_GUIDANCE(){}

void ASMC_GUIDANCE::Reset()
{
    error1 = 0.0;
    prev_error1 = 0.0;
    error2 = 0.0;
    prev_error2 = 0.0;
    error_i = 0.0;
    prev_error_i = 0.0;
    Uax = 0.0;
}

void ASMC_GUIDANCE::Manipulation(double _current)
{
    double sign = 0.0;
    prev_error1 = error1;
    prev_error2 = error2;
    prev_dot_K1 = dot_K1;

    error1 = set_point - _current_pos;
    error2 = 0.0       - _current_vel;

    if (controller_type == ANGULAR_DOF)
    {
        if (std::abs(error1) > PI)
        {
            error11 = (error1 / std::abs(error1)) * (std::abs(error1) - 2 * PI);
        }
        if (std::abs(error2) > PI)
        {
            error2 = (error2 / std::abs(error2)) * (std::abs(error2) - 2 * PI);
        }
    }

    s = error2 + lambda*error1;
    if (std::abs(s) - mu != 0.0)
    {
        sign = (s - mu) / (std::abs(s) - mu);
    } else
    {
        sign = 0;
    }
    dot_K1 = K1>K_min ?  K_alpha*sign:K_min;
    K1 += (dot_K1+prev_dot_K1)/2*sample_time_s;
    ua = -K1*sign - K2*s;
}