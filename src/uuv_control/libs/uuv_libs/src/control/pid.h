#pragma once 

struct PIDParams {
    double kP{0}, kI{0}, kD{0};
    double dt{0.01};

    double kUMax{1e9}, kUMin{-1e9};
    bool enable_ramp_rate_limit{false};
    double ramp_rate{1}; // units / second
};

class PID {
private:
    PIDParams params_;
    double prevError_{0};
    double setU_{0}; // Used to limit ramp rate.
    double integralError{0};

public:
    PID(const PIDParams& params): params_(params) {};

    double update(double measurement, double desired);
    static PIDParams defaultParams();

};