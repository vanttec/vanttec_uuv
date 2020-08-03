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

#include <vanttec_uuv/DetectedObstacles.h>
#include <geometry_msgs/Pose.h>

typedef enum MissionType_E
{
    NONE = 0,
    GATE = 1,
    BUOY = 2,
    TORPEDOES = 3,
} MissionType_E;

class MissionManager
{
    public:
    
        MissionType_E           current_mission;

        geometry_msgs::Pose     current_pose;
        

        MissionManager();
        ~MissionManager();
};

#endif // __UUV_MISSION_MANAGER__