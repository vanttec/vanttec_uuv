/** ----------------------------------------------------------------------------
 * @file: shootout_mission.cpp
 * @date: August 6, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Shootout mission class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __SHOOTOUT_MISSION__
#define __SHOOTOUT_MISSION__

#include <cmath>

#include <uuv_common.hpp>

#include <vanttec_uuv/GuidanceWaypoints.h>
#include <vanttec_uuv/DetectedObstacles.h>
#include <vanttec_uuv/Obstacle.h>
#include <geometry_msgs/Pose.h>


namespace ShootOutMission
{
    typedef enum ShootOutMissionStates_E
    {
        STANDBY = 0,
        INIT = 1,
        SWEEP = 2,
        ADVANCE = 3,
        CALCULATE = 4,
        PUBLISH = 5,
        NAVIGATE = 6,
        DONE = 7,
    } ShootOutMissionStates_E;

    class ShootOutMission
    {
        public:
            
            Side_E                          selected_side;
            ShootOutMissionStates_E         state_machine;
            ShootOutMissionStates_E         prev_state;

            int search_counter;
            int board_found;
            float alignment_position[2];

            vanttec_uuv::Obstacle            board;

            ShootOutMission();
            ~ShootOutMission();

            void UpdateStateMachine(Side_E* _side, 
                                    vanttec_uuv::DetectedObstacles* _obstacles,
                                    geometry_msgs::Pose* _pose,
                                    vanttec_uuv::GuidanceWaypoints* _waypoints);
        
        private:
            
            float init_adv_pos;
    };
}

#endif