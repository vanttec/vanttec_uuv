/** ----------------------------------------------------------------------------
 * @file: shootout_mission.cpp
 * @date: August 6, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Shootout mission class.
 * -----------------------------------------------------------------------------
 * */

#include "shootout_mission.hpp"

#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace uuv_common;

namespace ShootOutMission
{
    const float initial_depth       = -2.0;
    const float initial_heading     = -PI/2.0;
    const float depth_threshold     = 0.01;
    const float heading_threshold   = 0.01;

    ShootOutMission::ShootOutMission()
    {
        this->state_machine = STANDBY;
        this->selected_side = LEFT;
        this->search_counter = 0;
        this->board_found = -1;
        this->alignment_position[0] = 0;
        this->alignment_position[1] = 0;
    }

    ShootOutMission::~ShootOutMission(){}

    void ShootOutMission::UpdateStateMachine(Side_E* _side, 
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
                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();
                _waypoints->waypoint_list_z.clear();

                _waypoints->guidance_law = 0;
                _waypoints->waypoint_list_length = 2;
                _waypoints->waypoint_list_x = {0,0};
                _waypoints->waypoint_list_y = {0,0};
                
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
                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();
                _waypoints->waypoint_list_z.clear();

                _waypoints->guidance_law = 0;
                _waypoints->waypoint_list_length = 2;
                _waypoints->waypoint_list_x = {0,0};
                _waypoints->waypoint_list_y = {0,0};

                if (_obstacles->obstacles.size() > 0)
                {
                    for (int i = 0; i < _obstacles->obstacles.size(); i++)
                    {
                        if (_obstacles->obstacles[i].type == "r" ||
                            _obstacles->obstacles[i].type == "l")
                        {
                            this->board_found = i;
                            this->board = _obstacles->obstacles[i];
                            break;
                        }
                    }
                }

                if (this->board_found < 0)
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
                else
                {
                    this->state_machine = CALCULATE;
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
                        _waypoints->waypoint_list_x = {_pose->position.x, _pose->position.x + 4};
                        _waypoints->waypoint_list_y = {_pose->position.y, _pose->position.y + 1.5};
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
                this->search_counter = 0;

                float x_board_body = this->board.pose.position.x;
                float y_board_body = this->board.pose.position.y;   
                float z_board_ned  = this->board.pose.position.z;
               
                Eigen::Vector2d u;

                u << x_board_body,
                     y_board_body;
        
                Eigen::Matrix2d J;

                J  << cos(_pose->orientation.z), -1*sin(_pose->orientation.z),
                      sin(_pose->orientation.z), cos(_pose->orientation.z); 

                Eigen::MatrixXd rot = J * u;

                float x_board_ned = rot.coeff(0,0) + _pose->position.x;
                float y_board_ned = rot.coeff(1,0) + _pose->position.y;

                float alpha = this->board.pose.orientation.z;
                float alpha_prime = 0;

                switch((Side_E)*_side)
                {
                    case RIGHT:
                        if (this->board.type == "r")
                        {
                            alpha_prime = alpha + M_PI_2;
                        }
                        else
                        {
                            alpha_prime = alpha - M_PI_2;
                        }
                        break;
                    case LEFT:
                    default:
                        if (this->board.type == "l")
                        {
                            alpha_prime = alpha + M_PI_2;
                        }
                        else
                        {
                            alpha_prime = alpha - M_PI_2;
                        }
                        break;
                }

                float radius = 8 * this->board.radius;

                vanttec_uuv::GuidanceWaypoints _circle = GenerateCircle(radius,
                                                                        x_board_ned,
                                                                        y_board_ned, 
                                                                        this->board.pose.position.z,
                                                                        alpha);

                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();
                _waypoints->waypoint_list_z.clear();

                _waypoints->guidance_law = 2;
                _waypoints->waypoint_list_length = _circle.waypoint_list_length;
                _waypoints->waypoint_list_x = _circle.waypoint_list_x;
                _waypoints->waypoint_list_y = _circle.waypoint_list_y;
                _waypoints->waypoint_list_z = _circle.waypoint_list_z;

                this->alignment_position[0] = radius * cos(alpha_prime) + x_board_ned;
                this->alignment_position[1] = radius * sin(alpha_prime) + y_board_ned;

                std::cout << "align: " << this->alignment_position[0] << ", "  << this->alignment_position[1] << std::endl;       

                this->prev_state = CALCULATE;
                this->state_machine = PUBLISH;
                break;
            }
            case PUBLISH:
                this->state_machine = NAVIGATE;
                break;
            case NAVIGATE:
            {
                switch(this->prev_state)
                {
                    case ADVANCE:
                    {
                        float _euc_distance = pow(pow(_pose->position.x - _waypoints->waypoint_list_x[_waypoints->waypoint_list_length-1], 2) + 
                                              pow(_pose->position.y - _waypoints->waypoint_list_y[_waypoints->waypoint_list_length-1], 2), 
                                              0.5);
                        if (abs(_euc_distance) < 0.35)
                        {       
                            this->state_machine = INIT;
                        }
                        break;
                    }
                    case CALCULATE:
                    {
                        float _euc_distance = pow(pow(_pose->position.x - this->alignment_position[0], 2) + pow(_pose->position.y - this->alignment_position[1], 2), 0.5);
    
                        std::cout << "align error: " << _euc_distance << std::endl;       

                        if (abs(_euc_distance) < 0.25)
                        {
                            this->state_machine = DONE; 
                        }
                        break;
                    }
                    case SWEEP:
                        this->state_machine = ADVANCE;
                        break;
                    default:
                        break;
                }
                break;
            }
            case DONE:
            default:
                break;
        }
    }
}
