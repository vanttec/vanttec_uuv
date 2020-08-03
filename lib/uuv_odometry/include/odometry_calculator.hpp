/** ----------------------------------------------------------------------------
 * @file: odometry_calculator.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Odometry calculator class. Used to get velocities and positions 
 *         from the IMU.
 * -----------------------------------------------------------------------------
 * */

#ifndef __ODOMETRY_CALCULATOR_H__
#define __ODOMETRY_CALCULATOR_H__

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>

class OdometryCalculator
{
    public:
        
        /* Inputs from IMU */
        geometry_msgs::Vector3  linear_acceleration;
        geometry_msgs::Vector3  angular_rate;
        geometry_msgs::Vector3  angular_position;

        geometry_msgs::Vector3  prev_linear_acceleration;
        geometry_msgs::Vector3  prev_linear_velocity;
        geometry_msgs::Vector3  prev_angular_rate;

        /* Outputs to System */
        geometry_msgs::Pose     pose;
        geometry_msgs::Twist    twist;
        geometry_msgs::Accel    accel;

        /* Configuration */
        float sample_time_s;

        OdometryCalculator(float _sample_time);
        ~OdometryCalculator();
        
        void AccelPubCallback(const geometry_msgs::Vector3& _accel);
        void AngularRateCallback(const geometry_msgs::Vector3& _a_rate);
        void AngularPositionCallback(const geometry_msgs::Vector3& _a_pos);

        void UpdateParameters();

    private:

        double Integral(double x1, double x2, double c, float timestep);
        double Derivative(double x1, double x2, float timestep);
};

#endif