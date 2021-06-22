/** ----------------------------------------------------------------------------
 * @file: uuv_guidance_controller.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Guidance controller, which manages the different guidance laws 
 *         available to the UUV.
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_GUIDANCE_CONTROLLER_H__
#define __UUV_GUIDANCE_CONTROLLER_H__

#include <cmath>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <vanttec_uuv/GuidanceWaypoints.h>
#include <vanttec_uuv/MasterStatus.h>

/********** Helper Constants ***********/

const float     PI                      = 3.14159;
const uint8_t   LOS_WAYPOINT_OFFSET     = 2;

/********** Guidance Laws ***********/

typedef enum GuidanceLaws_E
{
    NONE = 0,
    LOS_GUIDANCE_LAW = 1,
    ORBIT_GUIDANCE_LAW = 2,
} GuidanceLaws_E;


/***************** 2D LOS ******************/

/* Enum for the 2D LOS Guidance Law States */

typedef enum LOSLawStates_E
{
    LOS_LAW_STANDBY = 0,
    LOS_LAW_DEPTH_NAV = 1,
    LOS_LAW_WAYPOINT_NAV = 2,
} LOSLawStates_E;

/* 2D LOS Guidance Law Struct */

typedef struct LOSLawStateMachine_S
{
    LOSLawStates_E      state_machine;
    int                 current_waypoint;      
} LOSLawStateMachine_S;

/***************** Orbit ******************/

typedef enum OrbitLawStates_E
{
    ORBIT_LAW_STANDBY = 0,
    ORBIT_LAW_DEPTH_NAV = 1,
    ORBIT_LAW_WAYPOINT_NAV = 2,
} OrbitLawStates_E;

/* Orbit Guidance Law Struct */

typedef struct OrbitLawStateMachine_S
{
    OrbitLawStates_E    state_machine;
    int                 current_waypoint;      
} OrbitLawStateMachine_S;

/********** Guidance Controller ***********/

class GuidanceController
{
    public:
        
        GuidanceLaws_E          current_guidance_law;
        LOSLawStateMachine_S    los_state_machine;
        OrbitLawStateMachine_S  orbit_state_machine;
        
        geometry_msgs::Pose                 current_positions_ned;
        geometry_msgs::Twist                desired_setpoints;
        vanttec_uuv::GuidanceWaypoints      current_waypoint_list;
        vanttec_uuv::MasterStatus           uuv_status;

        GuidanceController();
        ~GuidanceController();
        
        void OnCurrentPositionReception(const geometry_msgs::Pose& _pose);
        void OnWaypointReception(const vanttec_uuv::GuidanceWaypoints& _waypoints);
        void OnEmergencyStop(const std_msgs::Empty& _msg);
        void OnMasterStatus(const vanttec_uuv::MasterStatus& _status);

        void UpdateStateMachines();

    private:
        
        /* LOS Parameters */        
        float los_depth_error_threshold;
        float los_position_error_threshold;
        float los_lookahead_distance;
        float los_max_speed;
        float los_min_speed;
        float los_speed_gain;
        float los_euclidean_distance;

        /* Orbit Parameters */
        float orbit_depth_error_threshold;
        float orbit_position_error_threshold;
        float orbit_lookahead_distance;
        float orbit_max_speed;
        float orbit_min_speed;
        float orbit_speed_gain;
        float orbit_euclidean_distance;

};

#endif