/** ----------------------------------------------------------------------------
 * @file: gate_mission.cpp
 * @date: August 2, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Gate mission class.
 * -----------------------------------------------------------------------------
 * */

#include "gate_mission.hpp"

using namespace uuv_common;

namespace GateMission
{
    const float initial_depth       = -2.0;
    const float initial_heading     = -PI/2.0;
    const float depth_threshold     = 0.01;
    const float heading_threshold   = 0.01;

    GateMission::GateMission()
    {
        this->state_machine = STANDBY;
        this->selected_side = 0;
    }

    GateMission::~GateMission(){}

    void GateMission::UpdateStateMachine(Side_E& _side, 
                                         vanttec_uuv::DetectedObstacles& _obstacles, 
                                         geometry_msgs::Pose& _pose, 
                                         vanttec_uuv::GuidanceWaypoints& _waypoints)
    {
        switch(this->state_machine)
        {
            case INIT:
                _waypoints.depth_setpoint = -2;
                _waypoints.heading_setpoint = -PI/2;

                float depth_error = _pose - _waypoints.depth_setpoint;
                float heading_error = _pose - _waypoints.heading_setpoint;
                
                if (depth_error < depth_threshold && heading_error < heading_threshold)
                {
                    this->state_machine = SEARCH;
                }
                break;
            case SEARCH:
                for (float _heading = -PI/2; _heading < PI/2; _heading += PI)
                {
                    if (_obstacles.size() > 0)
                    {
                        for (int i = 0; i < _obstacles.size(); i++)
                        {
                            if (_obstacles[i].type == 0)
                            {
                                break;
                            }
                        }
                    }

                    _waypoints.heading_setpoint = _heading;
                }
                break;
            case CALCULATE:
                break;
            case NAVIGATE:
                break;
            case STANDBY:
            case DONE:
            default:
                break;
        }
    }
}
