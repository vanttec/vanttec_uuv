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

#include <std_msgs/Float32.h>
#include <cmath>

const float PI = 3.1416;

typedef enum DOFControllerType_E
{
    LINEAR_DOF_PID = 0,
    ANGULAR_DOF_PID = 1,
} DOFControllerType_E;

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
        
        float f_x;
        float g_x;

        DOFControllerType_E controller_type;
        
        PIDController(float _sample_time_s, const float _k_pid[3], const DOFControllerType_E _type);
        ~PIDController();
        
        void CalculateManipulation(float _current_value);
};

#endif