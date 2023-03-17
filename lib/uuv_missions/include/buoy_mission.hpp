/** ----------------------------------------------------------------------------
 * @file: buoy_mission.cpp
 * @date: August 2, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Buoy mission class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __BUOY_MISSION__
#define __BUOY_MISSION__

#include <cmath>

#include <uuv_common.hpp>

#include <vanttec_uuv/GuidanceWaypoints.h>
#include <vanttec_uuv/DetectedObstacles.h>
#include <vanttec_uuv/Obstacle.h>
#include <geometry_msgs/Pose.h>

namespace BuoyMission
{
    typedef enum BuoyMissionStates_E
    {
        STANDBY = 0,
        INIT = 1,
        SWEEP = 2,
        ADVANCE = 3,
        CALCULATE = 4,
        IDENTIFY = 5,
        RETRACT = 6,
        PUBLISH = 7,
        NAVIGATE = 8,
        DONE = 9,
        STOP = 10,
    } BuoyMissionStates_E;

    class BuoyMission
    {
        public:
            
            Side_E                           selected_side;
            BuoyMissionStates_E              state_machine;
            BuoyMissionStates_E              prev_state;

            int search_counter;
            int buoy_found;
            int current_buoy;
            int found_right_buoy;

            vanttec_uuv::Obstacle            first_buoy;
            vanttec_uuv::Obstacle            second_buoy;

            BuoyMission();
            ~BuoyMission();

            void UpdateStateMachine(Side_E* _side, 
                                    vanttec_uuv::DetectedObstacles* _obstacles,
                                    geometry_msgs::Pose* _pose,
                                    vanttec_uuv::GuidanceWaypoints* _waypoints);
        
        private:
            
            float init_adv_pos[2];
    };
}

#endif