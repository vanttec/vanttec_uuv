/** ----------------------------------------------------------------------------
 * @file: asmc.cpp
 * @date: June 17, 2021
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#include "asmc.hpp"

ASMC::ASMC(const float sample_time_s, const float lambda, const float K2, const float K_alpha, const float K1_init, const float K_min, const float mu, const DOFControllerType_E type)
{
    _sample_time_s = sample_time_s;
    _q_d = 0.0;
    _q_dot_d = 0.0;
    _error1 = 0.0;
    _error2 = 0.0;
    _prev_error1 = 0.0;
    _prev_error2 = 0.0;
    
    // Auxiliar control
    _ua = 0.0;

    // Sliding surface
    _lambda = lambda;
    _s = 0.0;

    // Gains
    _K1 = K1_init;
    _K2 = K2;
    _dot_K1 = 0.0;
    _prev_dot_K1 = 0.0;

    // Adaptive law
    _K_min = K_min;
    _K_alpha = K_alpha;
    _mu = mu;

    _controller_type = type;
}

ASMC::~ASMC(){}

void ASMC::Reset()
{
    _error1 = 0.0;
    _prev_error1 = 0.0;
    _error2 = 0.0;
    _prev_error2 = 0.0;
    _ua = 0.0;
    _K1 = 0.0;
}

void ASMC::UpdateSetPoint(const float q_d, const float q_dot_d)
{
    _q_d = q_d;
    _q_dot_d = q_dot_d;
}

void ASMC::CalculateAuxControl(float q, float q_dot)
{
    float sign = 0.0;
    _prev_error1 = _error1;
    _prev_error2 = _error2;
    _prev_dot_K1 = _dot_K1;

    _error1 = _q_d - q;
    _error2 = _q_dot_d - q_dot;

    if (_controller_type == ANGULAR_DOF)
    {
        if (fabs(_error1) > M_PI)
        {
            _error1 = (_error1 / fabs(_error1)) * (fabs(_error1) - 2 * M_PI);
        }
        if (fabs(_error2) > M_PI)
        {
            _error2 = (_error2 / fabs(_error2)) * (fabs(_error2) - 2 * M_PI);
        }
    }

    // Error 2 siendo la velocidad, error 1 se debe de integrar o est[a] bien poner directo la posici[o]n?
    _s = _error2 + _lambda*_error1;
    if (fabs(_s) - _mu != 0.0)
    {
        sign = (_s - _mu) / (fabs(_s) - _mu);
    } else
    {
        sign = 0;
    }

    _dot_K1 = _K1 > _K_min ?  _K_alpha*sign : _K_min;
    _K1 += (_dot_K1 + _prev_dot_K1) / 2 * _sample_time_s;
    _ua = -_K1*sign - _K2*_s;
}