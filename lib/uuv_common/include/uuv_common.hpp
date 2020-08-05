/** ----------------------------------------------------------------------------
 * @file: uuv_common.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Common functions and constants used throughout the uuv package.
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_COMMON_H__
#define __UUV_COMMON_H__

#include <vanttec_uuv/GuidanceWaypoints.h>
#include <cmath>

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
    vanttec_uuv::GuidanceWaypoints GenerateCircle(float _radius, float _x_center, float _y_center, float _z_center);
}

#endif
