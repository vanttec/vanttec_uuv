/** ----------------------------------------------------------------------------
 * @file: uuv_mission_manager.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Mission manager class, used to trigger and track mission progress.
 * -----------------------------------------------------------------------------
 * */

#ifndef __UUV_MISSION_MANAGER__
#define __UUV_MISSION_MANAGER__

#include "gate_mission.hpp"

#include <vanttec_uuv/DetectedObstacles.h>
#include <vanttec_uuv/GuidanceWaypoints.h>
#include <vanttec_uuv/MasterStatus.h>
#include <vanttec_uuv/MissionStatus.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>

using namespace uuv_common;

typedef enum MissionType_E
{
    NONE = 0,
    GATE = 1,
    BUOY = 2,
    TORPEDOES = 3,
    LOS_NAV_TEST = 4,
    ORBIT_NAV_TEST = 5,
} MissionType_E;

typedef enum Mode_E
{
    MANUAL = 0,
    AUTO = 1,
} Mode_E;

class MissionManager
{
    public:
    
        unsigned int                        ignore_e_stop;
        
        Side_E                              selected_side;
        Mode_E                              current_mode;       

        // Inputs
        geometry_msgs::Pose                 current_pose;
        vanttec_uuv::DetectedObstacles      detected_obstacles;

        // Outputs
        vanttec_uuv::GuidanceWaypoints      desired_waypoints;
        vanttec_uuv::MissionStatus          mission_status;

        // Missions
        GateMission::GateMission* gate_mission;

        MissionManager();
        ~MissionManager();

        void OnObstacleReception(const vanttec_uuv::DetectedObstacles& _obstacles);
        void OnMasterStatusReception(const vanttec_uuv::MasterStatus& _status);
        void OnMissionConfigReception(const vanttec_uuv::MissionStatus& _mission);
        void OnPoseReception(const geometry_msgs::Pose& _pose);
        void OnEStopReception(const std_msgs::Empty& _empty);

        void UpdateStateMachines();
    
    private:
        void ResetWaypoints();
};

#endif // __UUV_MISSION_MANAGER__