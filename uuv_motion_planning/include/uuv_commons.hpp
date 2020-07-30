#ifndef __UUV_COMMONS_H__
#define __UUV_COMMONS_H__

#include <uuv_guidance/GuidanceWaypoints.h>

#include <cmath>

namespace uuv_commons
{
    const float PI = 3.1416;

    uuv_guidance::GuidanceWaypoints GenerateCircle(float _radius, float _x_center, float _y_center, float _z_center)
    {
        uuv_guidance::GuidanceWaypoints _waypoints;
        
        float angle = 0;
        uint8_t counter = 0;
        
        while (angle <= uuv_commons::PI)
        {
            _waypoints.waypoint_list_x.push_back(_radius * std::sin(angle) + _x_center);
            _waypoints.waypoint_list_y.push_back(_radius * std::cos(angle) + _y_center);
            _waypoints.waypoint_list_z.push_back(_z_center);
            angle += uuv_commons::PI / 6;
            counter++;
        }
        
        angle = -uuv_commons::PI;

        while (angle <= 0)
        {
            _waypoints.waypoint_list_x.push_back(_radius * std::sin(angle) + _x_center);
            _waypoints.waypoint_list_y.push_back(_radius * std::cos(angle) + _y_center);
            _waypoints.waypoint_list_z.push_back(_z_center);
            angle += uuv_commons::PI / 6;
            counter++;
        }
        
        _waypoints.waypoint_list_length = counter;

        return _waypoints;
    }
}

#endif
