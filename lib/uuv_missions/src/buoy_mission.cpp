/** ----------------------------------------------------------------------------
 * @file: buoy_mission.cpp
 * @date: August 2, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Buoy mission class.
 * -----------------------------------------------------------------------------
 * */

#include "buoy_mission.hpp"

#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace uuv_common;

namespace BuoyMission
{
    const float initial_depth       = -2.0;
    const float initial_heading     = -PI/2.0;
    const float depth_threshold     = 0.01;
    const float heading_threshold   = 0.01;



    BuoyMission::GateMission()
    {
        this->state_machine = STANDBY;
        this->selected_side = LEFT;
        this->search_counter = 0;
        this->buoy_found = -1;
    }

    BuoyMission::~GateMission(){}

    void BuoyMission::UpdateStateMachine(Side_E* _side, 
                                         vanttec_uuv::DetectedObstacles* _obstacles, 
                                         geometry_msgs::Pose* _pose, 
                                         vanttec_uuv::GuidanceWaypoints* _waypoints)
    {

        switch(this->state_machine)
        {
            case STANDBY:
                this->state_machine = INIT;
                break;
            case INIT:
            {
                _waypoints->guidance_law = 0;
                _waypoints->depth_setpoint = -2;
                _waypoints->heading_setpoint = -5.0*PI/6.0;

                float depth_error = _pose->position.z - _waypoints->depth_setpoint;
                float heading_error = _pose->orientation.z - _waypoints->heading_setpoint;
                
                if (depth_error < depth_threshold && heading_error < heading_threshold)
                {
                    this->state_machine = SWEEP;
                }
                break;
            }
            case SWEEP:
            {
                if (_obstacles->obstacles.size() > 0)
                {
                    for (int i = 0; i < _obstacles->obstacles.size(); i++)
                    {
                        if (_obstacles->obstacles[i].type == "a" ||
                            _obstacles->obstacles[i].type == "b" )
                        {
                            switch(this->buoy_found)
                            {
                                case 0;
                                    this->first_buoy = _obstacle->obstacles[i];
                                    this->buoy_found++;
                                    break;
                                case 1:
                                    if (this->first_buoy.type != _obstacle->obstacles[i].type)
                                    {
                                        this->second_buoy = _obstacle->obstacles[i];
                                        this->buoy_found++;
                                    }
                                    break;
                                default:
                                    break;
                            }
                            break;
                        }
                    }
                }

                if (gate_found < 0)
                {
                    if (_waypoints->heading_setpoint >= 5.0*PI/6.0)
                    {
                        _waypoints->guidance_law = 0;
                        _waypoints->heading_setpoint = 0;
                        this->prev_state = SWEEP;
                        this->state_machine = PUBLISH;
                    }
                    else
                    {
                        _waypoints->guidance_law = 0;
                        _waypoints->heading_setpoint += PI/400.0;
                    }
                }
                else if (gate_found > 2)
                {
                    this->state_machine = CALCULATE;
                    this->buoy_found = -1;
                }                    
                break;
            }
            case ADVANCE:
                _waypoints->guidance_law = 1;
                _waypoints->waypoint_list_length = 2;

                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();

                switch(this->search_counter)
                {
                    case 0:
                        _waypoints->waypoint_list_x = {_pose->position.x, _pose->position.x + 3.35};
                        _waypoints->waypoint_list_y = {_pose->position.y, _pose->position.y + 1.85};
                        this->search_counter = 1;
                        break;
                    case 1:
                        _waypoints->waypoint_list_x = {_pose->position.x, _pose->position.x};
                        _waypoints->waypoint_list_y = {_pose->position.y, _pose->position.y - 3.35};
                        this->search_counter = 2;
                        break;
                    case 2:
                        _waypoints->waypoint_list_x = {_pose->position.x, _pose->position.x + 1.35};
                        _waypoints->waypoint_list_y = {_pose->position.y, _pose->position.y + 3.35};
                        this->search_counter = 1;
                        break;
                }

                _waypoints->waypoint_list_z = {_pose->position.z, _pose->position.z};
                
                this->prev_state = ADVANCE;
                this->state_machine = PUBLISH;
                break;
            case CALCULATE:
            {
                
                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();
                _waypoints->waypoint_list_z.clear();

                _waypoints->guidance_law = 1;
                _waypoints->waypoint_list_length = 4;
                _waypoints->waypoint_list_x = {_pose->position.x, x_wp_front, center_point_x, x_wp_back};
                _waypoints->waypoint_list_y = {_pose->position.y, y_wp_front, center_point_y, y_wp_back};
                _waypoints->waypoint_list_z = {this->gate.pose.position.z - 0.35,
                                              this->gate.pose.position.z - 0.35, 
                                              this->gate.pose.position.z - 0.35, 
                                              this->gate.pose.position.z - 0.35};

                this->prev_state = CALCULATE;
                this->state_machine = PUBLISH;
                break;
            }
            case PUBLISH:
                this->state_machine = NAVIGATE;
                break;
            case NAVIGATE:
            {
                float _euc_distance = pow(pow(_pose->position.x - _waypoints->waypoint_list_x[_waypoints->waypoint_list_length-1], 2) + 
                                          pow(_pose->position.y - _waypoints->waypoint_list_y[_waypoints->waypoint_list_length-1], 2), 
                                          0.5);

                if (abs(_euc_distance) < 0.35)
                {
                    switch(this->prev_state)
                    {
                        case ADVANCE:
                            this->state_machine = INIT;
                            break;
                        case CALCULATE:
                            this->state_machine = DONE; 
                            break;
                        case SWEEP:
                            this->state_machine = ADVANCE;
                            break;
                        default:
                            break;
                    }
                }
                break;
            }
            case DONE:
            default:
                break;
        }
    }
}
