/** ----------------------------------------------------------------------------
 * @file: odometry_calculator.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Odometry calculator class. Used to get velocities and positions 
 *         from the IMU.
 * -----------------------------------------------------------------------------
 * */

#include "odometry_calculator.hpp"

OdometryCalculator::OdometryCalculator(float _sample_time)
{
    this->sample_time_s = _sample_time;

    this->linear_acceleration.x = 0;
    this->linear_acceleration.y = 0;
    this->linear_acceleration.z = 0;

    this->angular_rate.x = 0;
    this->angular_rate.y = 0;
    this->angular_rate.z = 0;

    this->prev_linear_acceleration.x = 0;
    this->prev_linear_acceleration.y = 0;
    this->prev_linear_acceleration.z = 0;

    this->prev_angular_rate.x = 0;
    this->prev_angular_rate.y = 0;
    this->prev_angular_rate.z = 0;

    this->angular_position.x = 0;
    this->angular_position.y = 0;
    this->angular_position.z = 0;

    /* Output Init */

    /* Accelerations */
    this->accel.linear.x = 0;
    this->accel.linear.y = 0;
    this->accel.linear.z = 0;
    this->accel.angular.x = 0;
    this->accel.angular.x = 0;
    this->accel.angular.x = 0;

    /* Velocities */
    this->prev_linear_velocity.x = 0;;
    this->prev_linear_velocity.y = 0;;
    this->prev_linear_velocity.z = 0;;

    this->twist.linear.x = 0;
    this->twist.linear.y = 0;
    this->twist.linear.z = 0;
    this->twist.angular.x = 0;
    this->twist.angular.y = 0;
    this->twist.angular.z = 0;

    /* Pose */
    this->pose.position.x = 0;
    this->pose.position.y = 0;
    this->pose.position.z = 0;
    this->pose.orientation.x = 0;
    this->pose.orientation.y = 0;
    this->pose.orientation.z = 0;
    this->pose.orientation.w = 0;

}

OdometryCalculator::~OdometryCalculator(){}

void OdometryCalculator::AccelPubCallback(const geometry_msgs::Vector3& _accel)
{
    this->prev_linear_acceleration.x = this->linear_acceleration.x;
    this->prev_linear_acceleration.y = this->linear_acceleration.y;
    this->prev_linear_acceleration.z = this->linear_acceleration.z;
    
    this->linear_acceleration.x = _accel.x;
    this->linear_acceleration.y = _accel.y;
    this->linear_acceleration.z = _accel.z;
}

void OdometryCalculator::AngularRateCallback(const geometry_msgs::Vector3& _a_rate)
{
    this->prev_angular_rate.x = this->angular_rate.x;
    this->prev_angular_rate.y = this->angular_rate.y;
    this->prev_angular_rate.z = this->angular_rate.z;

    this->angular_rate.x = _a_rate.x;
    this->angular_rate.y = _a_rate.y;
    this->angular_rate.z = _a_rate.z;
}

void OdometryCalculator::AngularPositionCallback(const geometry_msgs::Vector3& _a_pos)
{
    this->angular_position.x = _a_pos.x;
    this->angular_position.y = _a_pos.y;
    this->angular_position.z = _a_pos.z;
}

void OdometryCalculator::UpdateParameters()
{
    /* Accelerations */
    this->accel.linear.x = this->linear_acceleration.x;
    this->accel.linear.y = this->linear_acceleration.y;
    this->accel.linear.z = this->linear_acceleration.z;
    this->accel.angular.x = OdometryCalculator::Derivative(this->prev_angular_rate.x, this->angular_rate.x, this->sample_time_s);
    this->accel.angular.x = OdometryCalculator::Derivative(this->prev_angular_rate.y, this->angular_rate.y, this->sample_time_s);
    this->accel.angular.x = OdometryCalculator::Derivative(this->prev_angular_rate.z, this->angular_rate.z, this->sample_time_s);

    /* Velocities */
    this->prev_linear_velocity.x = this->twist.linear.x;
    this->prev_linear_velocity.y = this->twist.linear.y;
    this->prev_linear_velocity.z = this->twist.linear.z;

    this->twist.linear.x = OdometryCalculator::Integral(this->prev_linear_acceleration.x, 
                                                        this->accel.linear.x, 
                                                        this->twist.linear.x, 
                                                        this->sample_time_s);
    this->twist.linear.y = OdometryCalculator::Integral(this->prev_linear_acceleration.y, 
                                                        this->accel.linear.y, 
                                                        this->twist.linear.y, 
                                                        this->sample_time_s);
    this->twist.linear.z = OdometryCalculator::Integral(this->prev_linear_acceleration.z, 
                                                        this->accel.linear.z, 
                                                        this->twist.linear.z, 
                                                        this->sample_time_s);
    this->twist.angular.x = this->angular_rate.x;
    this->twist.angular.y = this->angular_rate.y;
    this->twist.angular.z = this->angular_rate.z;

    /* Pose */
    this->pose.position.x = OdometryCalculator::Integral(this->prev_linear_velocity.x, 
                                                         this->twist.linear.x, 
                                                         this->pose.position.x, 
                                                         this->sample_time_s);
    this->pose.position.y = OdometryCalculator::Integral(this->prev_linear_velocity.y, 
                                                         this->twist.linear.y, 
                                                         this->pose.position.y, 
                                                         this->sample_time_s);
    this->pose.position.z = OdometryCalculator::Integral(this->prev_linear_velocity.z, 
                                                         this->twist.linear.z, 
                                                         this->pose.position.z, 
                                                         this->sample_time_s);

    this->pose.orientation.x = this->angular_position.x;
    this->pose.orientation.y = this->angular_position.y;
    this->pose.orientation.z = this->angular_position.z;
    this->pose.orientation.w = 0;
}

double OdometryCalculator::Integral(double x1, double x2, double c, float timestep)
{
    float integral_ = ((x1 + x2) / 2.0 * timestep) + c;
    return integral_;
}

double OdometryCalculator::Derivative(double x1, double x2, float timestep)
{
    float derivative_ = (x2 - x1) / timestep;
    return derivative_;
}

