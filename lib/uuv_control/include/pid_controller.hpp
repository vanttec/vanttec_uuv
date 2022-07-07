/** ----------------------------------------------------------------------------
 * @file: pid_controller.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: PID Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include "uuv_common.hpp"
#include "vanttec_uuv/EtaPose.h"
#include <std_msgs/Float32.h>
#include <cmath>

class PIDController
{
    public:
        float sample_time_s;
        
        float set_point;
        float manipulation;
        float error;
        float prev_error;

        float k_p;
        float k_i;
        float k_d;
        
        // float f_x;
        // float g_x;

        DOFControllerType_E controller_type;
        
        PIDController(const float _sample_time_s, const float _k_p, const float _k_i, const float _k_d, const DOFControllerType_E _type);
        ~PIDController();
        
        void UpdateSetPoint(const float& _set_point);
        void CalculateManipulation(const float _current_value);
};

#endif