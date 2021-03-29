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

#include <iostream>

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
        this->selected_side = LEFT;
        this->search_counter = 0;
    }

    GateMission::~GateMission(){}

    Eigen::Vector2d GateMission::BodyToNED(float* _u, float _angle, float* _offset)
    {
        Eigen::Vector2d u;

        u << _u[0],
             _u[1];
        
        Eigen::Matrix2d J;

        J  << cos(_angle), -1*sin(_angle),
              sin(_angle), cos(_angle); 

        Eigen::MatrixXd rot = (J * u);
        Eigen::Vector2d ned;
        std::cout << "AA" << std::endl;
        ned << (rot.coeff(0,0) + _offset[0]),
               (rot.coeff(1,0) + _offset[1]);
        std::cout << "BB" << std::endl;
        
        return ned;
    }


    void GateMission::UpdateStateMachine(Side_E* _side, 
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

                int gate_found = -1;
                if (_obstacles->obstacles.size() > 0)
                {
                    for (int i = 0; i < _obstacles->obstacles.size(); i++)
                    {
                        if (_obstacles->obstacles[i].type == "g")
                        {
                            gate_found = i;
                            this->gate = _obstacles->obstacles[i];
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
                /*
                float x_gate_body = this->gate.pose.position.x;
                float y_gate_body = this->gate.pose.position.y;   
                float z_gate_ned  = this->gate.pose.position.z;
               
                Eigen::Vector2d u;

                u << x_gate_body,
                     y_gate_body;
        
                Eigen::Matrix2d J;

                J  << cos(_pose->orientation.z), -1*sin(_pose->orientation.z),
                      sin(_pose->orientation.z), cos(_pose->orientation.z); 

                Eigen::MatrixXd rot = J * u;

                float x_gate_ned = rot.coeff(0,0) + _pose->position.x;
                float y_gate_ned = rot.coeff(1,0) + _pose->position.y;

                float center_point_x = 0;
                float center_point_y = 0;

                std::cout << "XSNED "<< _pose->position.x << " YSNED " << _pose->position.y << std::endl;
                std::cout << "XGNED "<< x_gate_ned << " YGNED " << y_gate_ned << std::endl;

                std::cout << *_side << std::endl;
                
                switch((Side_E)*_side)
                {
                    case RIGHT:
                        center_point_x = x_gate_ned + (0.45 * cos(this->gate.pose.orientation.z));
                        center_point_y = y_gate_ned + (0.45 * sin(this->gate.pose.orientation.z));
                        break;
                    case LEFT:
                    default:
                        center_point_x = x_gate_ned - (0.5 * cos(this->gate.pose.orientation.z));
                        center_point_y = y_gate_ned - (0.5 * sin(this->gate.pose.orientation.z));
                        break;
                }

                float x_wp_front = center_point_x - (2.5 * cos(this->gate.pose.orientation.z - PI/2.0));
                float x_wp_back = center_point_x + (3 * cos(this->gate.pose.orientation.z - PI/2.0));
                float y_wp_front = center_point_y - (2.5 * sin(this->gate.pose.orientation.z - PI/2.0));
                float y_wp_back = center_point_y + (3 * sin(this->gate.pose.orientation.z - PI/2.0));
                */

                float center_point[]  = {0, 0};
                float pose_offset[]   = {_pose->position.x, _pose->position.y};
                float psi = _pose->orientation.z;
                float alpha = -psi + this->gate.pose.orientation.z;

                std::cout << alpha << std::endl;

                switch((Side_E)*_side)
                {
                    case RIGHT:
                        center_point[0] = this->gate.pose.position.x + ((alpha / std::abs(alpha)) * (0.45 * cos(alpha)));
                        center_point[1] = this->gate.pose.position.y + ((alpha / std::abs(alpha)) * (0.45 * sin(alpha)));
                        break;
                    case LEFT:
                    default:
                        center_point[0] = this->gate.pose.position.x - ((alpha / std::abs(alpha)) * (0.5 * cos(alpha)));
                        center_point[1] = this->gate.pose.position.y - ((alpha / std::abs(alpha)) * (0.5 * sin(alpha)));
                        break;
                }
                
                float wp_front[]  = {center_point[0] + ((alpha / std::abs(alpha)) * (2.5 * cos(alpha + M_PI_2))), 
                                     center_point[1] + ((alpha / std::abs(alpha)) * (2.5 * sin(alpha + M_PI_2)))};
                float wp_back[]   = {center_point[0] - ((alpha / std::abs(alpha)) * (3 * cos(alpha + M_PI_2))),
                                     center_point[1] - ((alpha / std::abs(alpha)) * (3 * sin(alpha + M_PI_2)))};
                
                std::cout << wp_front[0] << ", " << wp_front[1] << std::endl;
                std::cout << center_point[0] << ", " << center_point[1] << std::endl;
                std::cout << wp_back[0] << ", " << wp_back[1] << std::endl;

                Eigen::Vector2d ned_front = this->BodyToNED(wp_front, _pose->orientation.z, pose_offset);
                Eigen::Vector2d ned_center = this->BodyToNED(center_point, _pose->orientation.z, pose_offset);
                Eigen::Vector2d ned_back = this->BodyToNED(wp_back, _pose->orientation.z, pose_offset);

                std::cout << ned_front[0] << ", " << ned_front[1] << std::endl;
                std::cout << ned_center[0] << ", " << ned_center[1] << std::endl;
                std::cout << ned_back[0] << ", " << ned_back[1] << std::endl;

                _waypoints->waypoint_list_x.clear();
                _waypoints->waypoint_list_y.clear();
                _waypoints->waypoint_list_z.clear();

                _waypoints->guidance_law = 1;
                _waypoints->waypoint_list_length = 4;
                _waypoints->waypoint_list_x = {_pose->position.x, 
                                               (float)ned_front[0], 
                                               (float)ned_center[0], 
                                               (float)ned_back[0]};
                std::cout << "G" << std::endl;

                _waypoints->waypoint_list_y = {_pose->position.y, 
                                               (float)ned_front[1], 
                                               (float)ned_center[1], 
                                               (float)ned_back[1]};

                _waypoints->waypoint_list_z = {this->gate.pose.position.z,
                                               this->gate.pose.position.z, 
                                               this->gate.pose.position.z, 
                                               this->gate.pose.position.z};

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
                            this->state_machine = DONE; 
                        }
                        break;
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
