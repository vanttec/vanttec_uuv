/** ----------------------------------------------------------------------------
 * @file: asmc_guidance.cpp
 * @date: July 20, 2021
 * @author: Carlos Medina
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ASMC Guidance Class
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
    error = 0.0;
    prev_error = 0.0;
    dot_error = 0.0;
    prev_dot_error = 0.0;
    error_i = 0.0;
    prev_error_i = 0.0;
    Uax = 0.0;
}

void ASMC_GUIDANCE::Manipulation(double _current)
{
    double sign = 0.0;
    prev_error = error;
    prev_dot_error = dot_error;
    prev_error_i = error_i;
    prev_dot_K1 = dot_K1;

    error = set_point - _current;
    dot_error = (error - prev_error)/sample_time_s;
    error_i += (error+prev_error)/2*sample_time_s;

    if (controller_type == ANGULAR_DOF)
    {
        if (std::abs(error) > PI)
        {
            error = (error / std::abs(error)) * (std::abs(error) - 2 * PI);
        }
        if (std::abs(dot_error) > PI)
        {
            dot_error = (dot_error / std::abs(dot_error)) * (std::abs(dot_error) - 2 * PI);
        }
    }

    sigma = error + Ka*error_i;
    if (std::abs(sigma) - miu != 0)
    {
        sign = (sigma - miu) / (std::abs(sigma) - miu);
    } else
    {
        sign = 0;
    }
    dot_K1 = K1>Kmin ?  Kalpha*sign:Kmin;
    K1 += (dot_K1+prev_dot_K1)/2*sample_time_s;

    if (sigma != 0)
    {
        sign = sigma / std::abs(sigma);
    } else
    {
        sign = 0;
    }
    Uax = -K1*std::sqrt(std::abs(sigma))*sign - K2*sigma;
}