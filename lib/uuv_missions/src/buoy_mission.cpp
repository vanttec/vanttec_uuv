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

    BuoyMission::BuoyMission()
    {
        this->state_machine = STANDBY;
        this->selected_side = LEFT;
        this->search_counter = 0;
        this->buoy_found = -1;
        this->current_buoy = 0;
        this->found_right_buoy = 0;
        this->init_adv_pos[0] = 0;
        this->init_adv_pos[1] = 0;
    }

    BuoyMission::~BuoyMission(){}

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
                            _obstacles->obstacles[i].type == "l" )
                        {
                            switch(this->buoy_found)
                            {
                                case -1:
                                {
                                    this->first_buoy = _obstacles->obstacles[i];

                                    float x_buoy_body = _obstacles->obstacles[i].pose.position.x;
                                    float y_buoy_body = _obstacles->obstacles[i].pose.position.y; 
                                
                                    Eigen::Vector2d u;

                                    u << x_buoy_body,
                                        y_buoy_body;
                            
                                    Eigen::Matrix2d J;

                                    J  << cos(_pose->orientation.z), -1*sin(_pose->orientation.z),
                                        sin(_pose->orientation.z), cos(_pose->orientation.z); 

                                    Eigen::MatrixXd rot = J * u;

                                    float x_buoy_ned = rot.coeff(0,0) + _pose->position.x;
                                    float y_buoy_ned = rot.coeff(1,0) + _pose->position.y;
                                    
                                    this->first_buoy.pose.position.x = x_buoy_ned;
                                    this->first_buoy.pose.position.y = y_buoy_ned;
                            
                                    this->buoy_found++;
                                    std::cout << "Found first buoy " << x_buoy_ned << ", " << y_buoy_ned << std::endl;
                                    break;
                                }
                                case 0:
                                {
                                    if (this->first_buoy.type != _obstacles->obstacles[i].type)
                                    {
                                        this->second_buoy = _obstacles->obstacles[i];

                                        float x_buoy_body = _obstacles->obstacles[i].pose.position.x;
                                        float y_buoy_body = _obstacles->obstacles[i].pose.position.y; 
                                    
                                        Eigen::Vector2d u;

                                        u << x_buoy_body,
                                            y_buoy_body;
                                
                                        Eigen::Matrix2d J;

                                        J  << cos(_pose->orientation.z), -1*sin(_pose->orientation.z),
                                            sin(_pose->orientation.z), cos(_pose->orientation.z); 

                                        Eigen::MatrixXd rot = J * u;

                                        float x_buoy_ned = rot.coeff(0,0) + _pose->position.x;
                                        float y_buoy_ned = rot.coeff(1,0) + _pose->position.y;
                                        
                                        this->second_buoy.pose.position.x = x_buoy_ned;
                                        this->second_buoy.pose.position.y = y_buoy_ned;
                                        
                                        this->buoy_found++;
                                        std::cout << "Found second buoy " << x_buoy_ned << ", " << y_buoy_ned << std::endl;
                                    }
                                    break;
                                }
                                default:
                                    break;
                            }
                            break;
                        }
                    }
                }

                if (this->buoy_found < 1)
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
                else if (this->buoy_found == 1)
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
                float delta_x = this->second_buoy.pose.position.x - this->first_buoy.pose.position.x;
                float delta_y = this->second_buoy.pose.position.y - this->first_buoy.pose.position.y;
                float radius  = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
                float alpha   = atan2(delta_y, delta_x);

                float alpha_dot = alpha + ((alpha/abs(alpha)) * M_PI_2);
                float _offset = (alpha_dot/abs(alpha_dot))*(abs(alpha_dot) - 2 * M_PI);

                float center_x = (radius/2 * cos(alpha)) + this->first_buoy.pose.position.x;
                float center_y = (radius/2 * sin(alpha)) + this->first_buoy.pose.position.y;

                std::cout << "Center: " << center_x << ", " << center_y << std::endl;
                std::cout << "Radius: " << radius << ", Angle: " << alpha << std::endl;
                
                vanttec_uuv::GuidanceWaypoints _circle = GenerateCircle(2*radius,
                                              center_x,
                                              center_y, 
                                              this->first_buoy.pose.position.z,
                                              _offset);

                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();
                _waypoints->waypoint_list_z.clear();

                _waypoints->guidance_law = 2;
                _waypoints->waypoint_list_length = _circle.waypoint_list_length;
                _waypoints->waypoint_list_x = _circle.waypoint_list_x;
                _waypoints->waypoint_list_y = _circle.waypoint_list_y;
                _waypoints->waypoint_list_z = _circle.waypoint_list_z;

                this->prev_state = CALCULATE;
                this->state_machine = PUBLISH;
                break;
            }
            case IDENTIFY:
            {               
                this->init_adv_pos[0] = _pose->position.x;
                this->init_adv_pos[1] = _pose->position.y;

                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();
                _waypoints->waypoint_list_z.clear();

                _waypoints->guidance_law = 1;
                _waypoints->waypoint_list_length = 2;

                switch(*_side)
                {
                    case RIGHT:
                        if (this->first_buoy.type == "r")
                        {
                            _waypoints->waypoint_list_x = {_pose->position.x, this->first_buoy.pose.position.x};
                            _waypoints->waypoint_list_y = {_pose->position.y, this->first_buoy.pose.position.y};
                            _waypoints->waypoint_list_z = {this->first_buoy.pose.position.z, this->first_buoy.pose.position.z};
                        }
                        else
                        {
                            _waypoints->waypoint_list_x = {_pose->position.x, this->second_buoy.pose.position.x};
                            _waypoints->waypoint_list_y = {_pose->position.y, this->second_buoy.pose.position.y};
                            _waypoints->waypoint_list_z = {this->second_buoy.pose.position.z, this->second_buoy.pose.position.z};
                        }
                        break;
                    case LEFT:
                    default:
                        if (this->first_buoy.type == "l")
                        {
                            _waypoints->waypoint_list_x = {_pose->position.x, this->first_buoy.pose.position.x};
                            _waypoints->waypoint_list_y = {_pose->position.y, this->first_buoy.pose.position.y};
                            _waypoints->waypoint_list_z = {this->first_buoy.pose.position.z, this->first_buoy.pose.position.z};
                        }
                        else
                        {
                            _waypoints->waypoint_list_x = {_pose->position.x, this->second_buoy.pose.position.x};
                            _waypoints->waypoint_list_y = {_pose->position.y, this->second_buoy.pose.position.y};
                            _waypoints->waypoint_list_z = {this->second_buoy.pose.position.z, this->second_buoy.pose.position.z};
                        }
                        break;
                }

                this->prev_state = IDENTIFY;
                this->state_machine = PUBLISH;

                break;
            }
            case RETRACT:
            {
                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();
                _waypoints->waypoint_list_z.clear();

                _waypoints->guidance_law = 0;
                _waypoints->waypoint_list_length = 2;
                _waypoints->waypoint_list_x = {-0.65, this->init_adv_pos[0]};
                _waypoints->waypoint_list_y = {0, this->init_adv_pos[1]};
                _waypoints->depth_setpoint = _pose->position.z;
                _waypoints->heading_setpoint = _pose->orientation.z;
                
                this->prev_state = RETRACT;
                this->state_machine = PUBLISH;
                
                break;
            }
            case STOP:
            {
                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();
                _waypoints->waypoint_list_z.clear();

                _waypoints->guidance_law = 0;
                _waypoints->waypoint_list_length = 2;
                _waypoints->waypoint_list_x = {0,0};
                _waypoints->waypoint_list_y = {0,0};
                _waypoints->depth_setpoint = _pose->position.z;
                _waypoints->heading_setpoint = _pose->orientation.z;
                
                this->prev_state = STOP;
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

                switch(this->prev_state)
                {
                    case ADVANCE:
                        if (abs(_euc_distance) < 0.35)
                        {       
                            this->state_machine = INIT;
                        }
                        break;
                    case CALCULATE:
                        if (abs(_euc_distance) < 0.35)
                        {
                            this->state_machine = IDENTIFY; 
                        }
                        break;
                    case IDENTIFY:
                        if (abs(_euc_distance) < 0.35)
                        {
                            this->state_machine = RETRACT; 
                        }
                        break;
                    case RETRACT:
                        if (abs(_euc_distance) < 0.35)
                        {
                            this->state_machine = STOP; 
                        }
                        break;
                    case SWEEP:
                        this->state_machine = ADVANCE;
                        break;
                    case STOP:
                        this->state_machine = DONE;
                        break;
                    default:
                        break;
                }
            }
            case DONE:
            default:
                break;
        }
    }
}
