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

namespace uuv_common
{
    /* Helper constants */
    const float PI = 3.1416;
    
    /* Helper functions */
    vanttec_uuv::GuidanceWaypoints GenerateCircle(float _radius, float _x_center, float _y_center, float _z_center, float _angle_offset);
    Eigen::Matrix6f                CalculateTransformation(double phi, double theta, double psi);
}

#endif
