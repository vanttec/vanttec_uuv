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
        int gate_found = -1;

        switch(this->state_machine)
        {
            case INIT:
                _waypoints.guidance_law = 0;
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
                if (_obstacles.size() > 0)
                {
                    for (int i = 0; i < _obstacles.size(); i++)
                    {
                        if (_obstacles[i].type == "g")
                        {
                            gate_found = i;
                            break;
                        }
                    }
                }

                if (gate_found < 0)
                {
                    if (_waypoints.heading_setpoint > PI/2.0)
                    {
                        _waypoints.guidance_law = 1;
                        _waypoints.waypoint_list_length = 2;
                        _waypoints.waypoint_list_x = {_pose.position.x, _pose.position.x + 0.5};
                        _waypoints.waypoint_list_y = {_pose.position.y, _pose.position.z};
                        _waypoints.waypoint_list_z = {_pose.position.z, _pose.position.z};
                        _waypoints.heading_setpoint = -PI/2.0;
                    }
                    else
                    {
                        _waypoints.guidance_law = 0;
                        _waypoints.heading_setpoint += PI/800.0;
                    }
                }
                else
                {
                    this->state_machine = CALCULATE;
                    continue;
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
