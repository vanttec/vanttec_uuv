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

ASMC::ASMC(const double _K2, const double _K_alpha, const double _K_min, const double _mu, const DOFControllerType_E _type)
{
    K1 = 0;
    dot_K1 = 0;
    K2 = _K2;
    K_alpha = _K_alpha;
    K_min = _K_min;
    mu = mu;
    controller_type = _type;
    error1 = 0.0;
    error2 = 0.0;
    ua = 0.0;
}

ASMC::~ASMC(){}

void ASMC::Reset()
{
    error1 = 0.0;
    prev_error1 = 0.0;
    error2 = 0.0;
    prev_error2 = 0.0;
    ua = 0.0;
}

void ASMC::SetAdaptiveParams(const double _K_min, const double _K_alpha, const double _mu)
{
    K_alpha = _K_alpha;
    K_min = _K_min;
    mu = _mu;
}

void ASMC::CalculateAuxControl(double set_point, double _current_pos, double _current_vel, double _sample_time_s)
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
            error1 = (error1 / std::abs(error1)) * (std::abs(error1) - 2 * PI);
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