 #include "pid.hpp"
 #include <algorithm>
 #include <cmath>
 
 PID::PID() { }

 PID::PID(const PIDParameters &params) { params_ = params; }
 
 double PID::update(double measurement, double desired) {
   double error = desired - measurement;
 
   double d = (error - prev_error_) / params_.kDt;
   double i = ((error + prev_error_) / 2 * params_.kDt) + error;
   prev_error_ = error;
 
   double u = params_.kP * error + params_.kI * i + params_.kD * d;
 
   // If ramp rate is disabled, or if we are within ramp rate, go to U.
   if (!params_.enable_ramp_rate_limit ||
       std::abs((set_u_ - u) * params_.kDt) < params_.ramp_rate) {
         set_u_ = u;
   } else {
     // Ramp rate is enabled, and we can only increase by ramp rate.
     set_u_ += std::copysign(params_.ramp_rate * params_.kDt, u - set_u_);
   }
 
   return std::clamp(set_u_, params_.kUMin, params_.kUMax);
 }

 void PID::clear(){
  prev_error_ = 0.0;
 }