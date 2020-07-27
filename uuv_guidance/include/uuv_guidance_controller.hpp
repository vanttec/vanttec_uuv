#ifndef __UUV_GUIDANCE_CONTROLLER_H__
#define __UUV_GUIDANCE_CONTROLLER_H__

#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <uuv_guidance/GuidanceWaypoints.h>

/********** Guidance Laws ***********/

typedef enum
{
    NONE = 0,
    LOS_GUIDANCE_LAW = 1,
    ORBIT_GUIDANCE_LAW = 2,
} GuidanceLaws_E GuidanceLaws_E;


/***************** 2D LOS ******************/

/* Enum for the 2D LOS Guidance Law States */

typedef enum 
{
    LOS_LAW_STANDBY = 0,
    LOS_LAW_DEPTH_NAV = 1,
    LOS_LAW_WAYPOINT_NAV = 2,
} LOSLawStates_E LOSLawStates_E;

/* 2D LOS Guidance Law Struct */

typedef struct 
{
    LOSLawStates_E      state_machine;
    uint8_t             current_waypoint;      
} LOSLawStateMachine_S LOSLawStateMachine_S;

/***************** Orbit ******************/


/********** Guidance Controller ***********/

class GuidanceController
{
    public:
        
        GuidanceLaws_E          current_guidance_law;
        LOSLawStateMachine_S    los_state_machine;
        
        geometry_msgs::Pose                 current_positions_ned;
        geometry_msgs::Twist                desired_setpoints;
        uuv_guidance::GuidanceWaypoints     current_waypoint_list;

        GuidanceController();
        ~GuidanceController();
        
        void OnCurrentPositionReception(const geometry_msgs::Pose& _pose);
        void OnWaypointReception(const uuv_guidance::GuidanceWaypoints& _waypoints);
        void OnEmergencyStop(const std_msgs::Empty& _msg);

        void UpdateStateMachines();

    private:
        
        /* LOS Functions and Parameters */        
        void ComputeDesiredValues_LOS();
        float los_depth_error_threshold;
        float los_position_error_threshold;
        float los_lookahead_distance;

        /* Orbit Functions and Parameters */        
        void ComputeDesiredValues_Orbit();
};

#endif