/** ----------------------------------------------------------------------------
 * @file: uuv_common.hpp
 * @date: March 2, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Common functions and constants used throughout the uuv package.
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_COMMON_H__
#define __UUV_COMMON_H__

#include <vanttec_uuv/GuidanceWaypoints.h>
#include <cmath>
#include <eigen3/Eigen/Dense>

typedef enum Side_E
{
    LEFT = 0,
    RIGHT = 1,
} Side_E;

typedef enum DOFControllerType_E
{
    LINEAR_DOF = 0,
    ANGULAR_DOF = 1,
} DOFControllerType_E;

namespace uuv_common
{
    /* Helper constants */
    static const float rho = 1000;
    static const float g = 9.81;
    
    /* Helper functions */
    vanttec_uuv::GuidanceWaypoints GenerateCircle(float _radius, float _x_center, float _y_center, float _z_center, float _angle_offset);
    // Eigen::MatrixXf                CalculateTransformation(double phi, double theta, double psi);
    Eigen::MatrixXf CalculateRotation(double phi, double theta, double psi);
}

#endif
