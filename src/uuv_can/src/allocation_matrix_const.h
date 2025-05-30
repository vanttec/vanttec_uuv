#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>

// Constants
constexpr double T200_THRUST_N = 39.7;

constexpr double PHI = 30.0 * M_PI / 180.0;
constexpr double THETA = 0.0;

constexpr double r_f1 = 0.15, r_f2 = 0.10, r_f3 = 0.05;
constexpr double r_m1 = 0.20, r_m2 = 0.10;
constexpr double r_t1 = 0.12, r_t2 = 0.10, r_t3 = 0.04;

// Function to initialize the allocation matrix
inline void fill_allocation_matrix(Eigen::MatrixXd& coef) {
    coef(0,0) = -T200_THRUST_N * sin(PHI) * cos(THETA);
    coef(1,0) = -T200_THRUST_N * sin(PHI) * sin(THETA);
    coef(2,0) =  T200_THRUST_N * cos(PHI);
    coef(3,0) = -T200_THRUST_N * sin(PHI) * sin(THETA) * r_f3 - T200_THRUST_N * cos(PHI) * r_f2;
    coef(4,0) =  T200_THRUST_N * cos(PHI) * r_f1 + T200_THRUST_N * sin(PHI) * cos(THETA) * r_f3;
    coef(5,0) =  T200_THRUST_N * sin(PHI) * (sin(THETA) * r_f1 - cos(THETA) * r_f2);

    coef(0,1) = -T200_THRUST_N * sin(PHI) * cos(THETA);
    coef(1,1) =  T200_THRUST_N * sin(PHI) * sin(THETA);
    coef(2,1) =  T200_THRUST_N * cos(PHI);
    coef(3,1) =  T200_THRUST_N * sin(PHI) * sin(THETA) * r_f3 - T200_THRUST_N * cos(PHI) * r_f2;
    coef(4,1) =  T200_THRUST_N * cos(PHI) * r_f1 + T200_THRUST_N * sin(PHI) * cos(THETA) * r_f3;
    coef(5,1) =  T200_THRUST_N * sin(PHI) * (sin(THETA) * r_f1 - cos(THETA) * r_f2);

    coef(0,2) =  T200_THRUST_N * sin(PHI) * cos(THETA);
    coef(1,2) = -T200_THRUST_N * sin(PHI) * sin(THETA);
    coef(2,2) =  T200_THRUST_N * cos(PHI);
    coef(3,2) = -T200_THRUST_N * sin(PHI) * sin(THETA) * r_t3 - T200_THRUST_N * cos(PHI) * r_t2;
    coef(4,2) =  T200_THRUST_N * cos(PHI) * r_t1 - T200_THRUST_N * sin(PHI) * cos(THETA) * r_t3;
    coef(5,2) =  T200_THRUST_N * sin(PHI) * (cos(THETA) * r_t2 + sin(THETA) * r_t1);

    coef(0,3) =  T200_THRUST_N * sin(PHI) * cos(THETA);
    coef(1,3) =  T200_THRUST_N * sin(PHI) * sin(THETA);
    coef(2,3) =  T200_THRUST_N * cos(PHI);
    coef(3,3) =  T200_THRUST_N * sin(PHI) * sin(THETA) * r_t3 - T200_THRUST_N * cos(PHI) * r_t2;
    coef(4,3) =  T200_THRUST_N * cos(PHI) * r_t1 - T200_THRUST_N * sin(PHI) * cos(THETA) * r_t3;
    coef(5,3) =  T200_THRUST_N * sin(PHI) * (cos(THETA) * r_t2 - sin(THETA) * r_t1);

    coef(0,4) = 0.0;
    coef(1,4) = 0.0;
    coef(2,4) = -T200_THRUST_N;
    coef(3,4) =  T200_THRUST_N * r_m2;
    coef(4,4) = -T200_THRUST_N * r_m1;
    coef(5,4) = 0.0;

    coef(0,5) = 0.0;
    coef(1,5) = 0.0;
    coef(2,5) = -T200_THRUST_N;
    coef(3,5) =  T200_THRUST_N * r_m2;
    coef(4,5) = -T200_THRUST_N * r_m1;
    coef(5,5) = 0.0;
}
