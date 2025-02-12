#pragma once

#include "pid.h"
#include <eigen3/Eigen/Dense>
#include <vector>

class PID6DOF {
private:
    Eigen::VectorXf u_;
    std::array<PID,6> pids_; // {x,y,z,phi,theta,psi}

    // PID pidX_{PID::defaultParams()};
    // PID pidY_{PID::defaultParams()};
    // PID pidZ_{PID::defaultParams()};
    // PID pidPhi_{PID::defaultParams()}; // roll
    // PID pidTheta_{PID::defaultParams()}; // pitch
    // PID pidPsi_{PID::defaultParams()}; // yaw

public:
    PID6DOF(const std::array<PIDParams, 6>& params) 
                    : pids_{{params[0], params[1], params[2], 
                    params[3], params[4], params[5]}} {}   
    
    Eigen::VectorXf update(const Eigen::VectorXf& measurement, 
                            const Eigen::VectorXf& desired);
};

Eigen::VectorXf PID6DOF::update(const Eigen::VectorXf& measurement, 
                                const Eigen::VectorXf& desired) {
    Eigen::VectorXf error = desired - measurement;
    for (int i = 0; i < 6; i++) {
        u_(i) = pids_[i].update(measurement(i), desired(i));
    }
    
    return u_;
}