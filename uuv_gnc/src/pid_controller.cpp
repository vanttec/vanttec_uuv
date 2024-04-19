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

PIDController::PIDController(float _sample_time_s, const float _k_pid[3], const DOFControllerType_E _type)
{
    this->sample_time_s     = _sample_time_s;
    this->k_p               = _k_pid[0];
    this->k_i               = _k_pid[1];
    this->k_d               = _k_pid[2];

    this->error             = 0;
    this->prev_error        = 0;
    this->set_point         = 0;
    this->manipulation      = 0;
    
    this->f_x               = 0;
    this->g_x               = 0;

    this->controller_type   = _type;
}

PIDController::~PIDController(){}

void PIDController::CalculateManipulation(float _current_value)
{
    this->prev_error    = this->error;
    this->error         = this->set_point - _current_value;

    if (this->controller_type == ANGULAR_DOF_PID)
    {
        if (std::abs(this->error) > PI)
        {
            this->error = (this->error / std::abs(this->error)) * (std::abs(this->error) - 2 * PI);
        }
    }

    float error_d       = (this->error - this->prev_error) / this->sample_time_s;
    float error_i       = ((this->error + this->prev_error) / 2 * this->sample_time_s) + this->error;

    this->manipulation  = (1 / this->g_x) * (-this->f_x + this->k_p * this->error + this->k_i * error_i + this->k_d * error_d);
}