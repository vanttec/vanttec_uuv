/** ----------------------------------------------------------------------------
 * @file: gate_mission.cpp
 * @date: August 2, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Gate mission class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __CHOOSE_SIDE_MISSION__
#define __CHOOSE_SIDE_MISSION__

#include <cmath>

#include <uuv_common.hpp>

#include <vanttec_uuv/GuidanceWaypoints.h>
#include <vanttec_uuv/DetectedObstacles.h>
#include <vanttec_uuv/Obstacle.h>
#include <geometry_msgs/Pose.h>

typedef enum Side_E
{
    LEFT = 0,
    RIGHT = 1,
} Side_E;

namespace GateMission
{
    typedef enum GateMissionStates_E
    {
        STANDBY = 0,
        INIT = 1,
        SWEEP = 2,
        ADVANCE = 3,
        CALCULATE = 4,
        PUBLISH = 5,
        NAVIGATE = 6,
        DONE = 7,
    } GateMissionStates_E;

    class GateMission
    {
        public:
            
            Side_E                           selected_side;
            GateMissionStates_E              state_machine;
            GateMissionStates_E              prev_state;

            int search_counter;

            vanttec_uuv::Obstacle            gate;

            GateMission();
            ~GateMission();

            void UpdateStateMachine(Side_E* _side, 
                                    vanttec_uuv::DetectedObstacles* _obstacles,
                                    geometry_msgs::Pose* _pose,
                                    vanttec_uuv::GuidanceWaypoints* _waypoints);
        
        private:
            
            float init_adv_pos;
    };
}

#endif