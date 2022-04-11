/** ----------------------------------------------------------------------------
 * @file: pid_controller.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: PID Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#include "pid_controller.hpp"

PIDController::PIDController(const float _sample_time_s, const float _k_p, const float _k_i, const float _k_d, const DOFControllerType_E _type)
{
    sample_time_s     = _sample_time_s;
    k_p               = _k_p;
    k_i               = _k_i;
    k_d               = _k_d;

    error             = 0;
    prev_error        = 0;
    set_point         = 0;
    manipulation      = 0;
    
    // f_x               = 0;
    // g_x               = 0;

    controller_type   = _type;
}

PIDController::~PIDController(){}

void PIDController::CalculateManipulation(const float _current_value)
{
    prev_error    = error;
    error         = set_point - _current_value;

    if (controller_type == ANGULAR_DOF)
    {
        if (std::abs(error) > M_PI)
        {
            // error = error - ((error / std::abs(error)) * 2 * M_PI);
            error = (error / std::abs(error)) * (std::abs(error) - 2 * M_PI);
        }
    }

    const float error_d       = (error - prev_error) / sample_time_s;
    const float error_i       = ((error + prev_error) / 2 * sample_time_s) + error;

    // manipulation  = (1 / g_x) * (-f_x + k_p * error + k_i * error_i + k_d * error_d);
    manipulation  = k_p * error + k_i * error_i + k_d * error_d;
}

void PIDController::UpdateSetPoint(const float& _set_point)
{
    set_point = _set_point;
}