#ifndef __UUV_COMMON_H__
#define __UUV_COMMON_H__

#include <vanttec_uuv/GuidanceWaypoints.h>
#include <cmath>

namespace uuv_common
{
    /* Helper constants */
    const float PI = 3.1416;
    
    /* Helper functions */
    vanttec_uuv::GuidanceWaypoints GenerateCircle(float _radius, float _x_center, float _y_center, float _z_center);
}

#endif
