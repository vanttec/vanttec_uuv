#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include <std_msgs/Float32.h>

class PIDController
{
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
    
    PIDController(float _sample_time_s, float _k_p, float _k_i, float _k_d);
    ~PIDController();

    void UpdateSetPoint(const std_msgs::Float32& _set_point);
    void CalculateManipulation(float _current_value);
};

#endif