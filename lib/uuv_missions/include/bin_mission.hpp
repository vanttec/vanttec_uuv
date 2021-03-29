/** ----------------------------------------------------------------------------
 * @file: bin_mission.cpp
 * @date: September 1, 2020
 * @author: Edison Altamirano
 * @email: edison.altamirano@tec.mx
 * 
 * @brief: Bin mission class.
 * -----------------------------------------------------------------------------
 * */

#ifndef __BIN_MISSION__
#define __BIN_MISSION__

#include <cmath>

#include <uuv_common.hpp>

#include <vanttec_uuv/GuidanceWaypoints.h>
#include <vanttec_uuv/DetectedObstacles.h>
#include <vanttec_uuv/Obstacle.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Dense>

namespace BinMission
{
    typedef enum BinMissionStates_E
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
    } BinMissionStates_E;

    class BinMission
    {
        public:
            
            Side_E                          selected_side;
            BinMissionStates_E              state_machine;
            BinMissionStates_E              prev_state;

            int search_counter;
            int buoy_found;
            int current_buoy;
            int found_right_buoy;

            vanttec_uuv::Obstacle            square;

            BinMission();
            ~BinMission();

            void UpdateStateMachine(Side_E* _side, 
                                    vanttec_uuv::DetectedObstacles* _obstacles,
                                    geometry_msgs::Pose* _pose,
                                    vanttec_uuv::GuidanceWaypoints* _waypoints);
        
            Eigen::Vector2d BodyToNED(float* _u, float _angle, float* _offset);
        
        private:
            
            float init_adv_pos[2];
    };
}

#endif