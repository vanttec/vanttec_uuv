#include <eigen3/Eigen/Dense>
#include <array>
#include <iostream>

#pragma once
namespace uuv{
  // struct ControllerOutput {
  //   // Main output data, left and right thruster
  //   double left_thruster, right_thruster;
  //   double Tx, Tz;
  // };

  // struct ControllerState {
  //   double u, v, r;
  //   double psi;
  // };
  
  struct UUVState{
    Eigen::VectorXf eta;
    Eigen::VectorXf eta_dot;
    Eigen::VectorXf eta_dot_prev;
    Eigen::VectorXf nu;
    Eigen::VectorXf nu_dot;
    Eigen::VectorXf nu_dot_prev;
  };
}