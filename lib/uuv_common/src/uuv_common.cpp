/** ----------------------------------------------------------------------------
 * @file: uuv_common.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Common functions and constants used throughout the uuv package.
 * -----------------------------------------------------------------------------
 * */

#include "uuv_common.hpp"

namespace uuv_common
{
    vanttec_uuv::GuidanceWaypoints GenerateCircle(float _radius, float _x_center, float _y_center, float _z_center, float _angle_offset)
    {
        vanttec_uuv::GuidanceWaypoints _waypoints;
        
        float angle = _angle_offset;
        uint8_t counter = 0;
        
        while (angle <= (_angle_offset + uuv_common::PI))
        {
            _waypoints.waypoint_list_x.push_back(_radius * std::cos(angle) + _x_center);
            _waypoints.waypoint_list_y.push_back(_radius * std::sin(angle) + _y_center);
            _waypoints.waypoint_list_z.push_back(_z_center);
            angle += uuv_common::PI / 6;
            counter++;
        }
        
        angle = _angle_offset - uuv_common::PI;

        while (angle <= _angle_offset)
        {
            _waypoints.waypoint_list_x.push_back(_radius * std::cos(angle) + _x_center);
            _waypoints.waypoint_list_y.push_back(_radius * std::sin(angle) + _y_center);
            _waypoints.waypoint_list_z.push_back(_z_center);
            angle += uuv_common::PI / 6;
            counter++;
        }
        
        _waypoints.waypoint_list_length = counter;

        return _waypoints;
    }
}