/** ----------------------------------------------------------------------------
 * @file: waypoint_publisher.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Waypoint publisher class, used to test waypoint navigation.
 * -----------------------------------------------------------------------------
 * */

#ifndef __WAYPOINT_PUBLISHER_H__
#define __WAYPOINT_PUBLISHER_H__

#include <uuv_common.hpp>
#include <vanttec_uuv/GuidanceWaypoints.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>

class WaypointPublisher
{
    public:

        uint8_t trajectory_selector;
        uint8_t path_publish_flag;
        float   circle_radius;

        vanttec_uuv::GuidanceWaypoints  waypoints;
        nav_msgs::Path                  path;

        WaypointPublisher();
        ~WaypointPublisher();

        void OnTrajectoryReceive(const std_msgs::UInt8& _trajectory);
        void OnRadiusReceive(const std_msgs::Float32& _radius);
        void WaypointSelection();

};

#endif