#pragma once

#include <limits>

struct PIDParameters {
    double kP{0}, kI{0}, kD{0};
    double kDt{0.01};
    
    double kUMax{1e9};
    double kUMin{-1e9};

    bool enable_ramp_rate_limit{false};
    double ramp_rate{1}; // units / second
};

class PID {
public:
  PID();
  PID(const PIDParameters &params);

  double update(double measurement, double desired);

  static PIDParameters defaultParams();

  void clear();

private:
  PIDParameters params_;
  double prev_error_{0};

  double set_u_{0}; // Used to limit ramp rate.
};