#include "pid.h"
#include <algorithm>
#include <cmath>

double PID::update(double measurement, double desired) {
    double error = desired - measurement;
    double p = params_.kP * error;
    integralError += error * params_.dt;
    double i = params_.kI * integralError;
    double d = params_.kD * (error - prevError_)/ params_.dt;
    prevError_ = error;

    double u = p + i + d;
  // If ramp rate is disabled, or if we are within ramp rate, go to U.
  if (!params_.enable_ramp_rate_limit ||
      std::abs((setU_ - u)) < params_.ramp_rate * params_.dt) {
        setU_ = u;
  } else {
    // Ramp rate is enabled, and we can only increase by ramp rate.
    setU_ += std::copysign(params_.ramp_rate * params_.dt, u - setU_);
  }

  return std::clamp(setU_, params_.kUMin, params_.kUMax);
}

PIDParams PID::defaultParams() {
  PIDParams p{};
  p.kD = 0.0;
  p.kD = 0.0;
  p.dt = 0.0;
  p.kUMax = 0.0;
  p.kUMin = 0.0;
  p.enable_ramp_rate_limit = false;
  p.ramp_rate = 0.0;
  return p;
}