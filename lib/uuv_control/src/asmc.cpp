/** ----------------------------------------------------------------------------
 * @file: asmc.cpp
 * @date: June 17, 2021
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#include "asmc.hpp"

ASMC::ASMC(double _sample_time_s, const double _K2, const double _Kalpha, const double _Kmin, const double _miu, const DOFControllerType_E _type)
{
    sample_time_s = _sample_time_s;
    K1 = 0;
    dot_K1 = 0;
    K2 = _K2;
    Kalpha = _Kalpha;
    Kmin = _Kmin;
    miu = miu;
    controller_type = _type;
    error = 0.0;
    dot_error = 0.0;
    manipulation = 0.0;
}

ASMC::~ASMC(){}

void ASMC::Reset()
{
    error = 0.0;
    prev_error = 0.0;
    dot_error = 0.0;
    prev_dot_error = 0.0;
    manipulation = 0.0;
}

void ASMC::SetAdaptiveParams(const double _Kmin, const double _Kalpha, const double _miu)
{
    Kalpha = _Kalpha;
    Kmin = _Kmin;
    miu = miu;
}

void ASMC::Manipulation(double _current)
{
    double sign = 0.0;
    prev_error = error;
    prev_dot_error = dot_error;
    prev_dot_K1 = dot_K1;

    error = set_point - _current;
    dot_error = (error - prev_error)/sample_time_s;

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

    sigma = dot_error + lambda*error;
    if (std::abs(sigma) - miu != 0)
    {
        sign = (std::abs(sigma) - miu) / (std::abs(sigma) - miu);
    } else
    {
        sign = 0;
    }
    dot_K1 = K1>Kmin ?  Kalpha*sign:Kmin;
    K1 += (dot_K1+prev_dot_K1)/2*sample_time_s;
    manipulation = K1*sign + K2*sigma; // Checar sign
}