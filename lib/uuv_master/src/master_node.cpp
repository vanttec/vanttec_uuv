/** ----------------------------------------------------------------------------
 * @file: master_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Master node, in charge of receiving control input and managing other
 *         nodes in the architecture.
 * -----------------------------------------------------------------------------
 **/

#include "master_node.hpp"

UUVMasterNode::UUVMasterNode(const float _default_speed)
{
    this->default_speed             = _default_speed;
    this->e_stop_flag               = 0;
    this->mission_debounce          = 0;
    this->mission.current_mission   = 0;
    this->mission.selected_side     = 0;
    this->status.status             = 0;
}

UUVMasterNode::~UUVMasterNode(){}

void UUVMasterNode::keyboardUpCallback(const vehicle_user_control::KeyboardKey& msg)
{
    switch(msg.code)
    {
        case vehicle_user_control::KeyboardKey::KEY_w:
        case vehicle_user_control::KeyboardKey::KEY_a:
        case vehicle_user_control::KeyboardKey::KEY_s:
        case vehicle_user_control::KeyboardKey::KEY_d:
            this->velocities.angular.x      = 0.0;      //Roll
            this->velocities.angular.y      = 0.0;      //Pitch
            this->velocities.linear.x       = 0.0;      //x linear movement
            this->velocities.linear.y       = 0.0;      //y linear movement
            break;
    }
}

void UUVMasterNode::keyboardDownCallback(const vehicle_user_control::KeyboardKey& msg)
{
    switch(msg.code)
    {
        case vehicle_user_control::KeyboardKey::KEY_SPACE:
            this->velocities.angular.x      = 0.0;      //Roll
            this->velocities.angular.y      = 0.0;      //Pitch
            this->velocities.linear.x       = 0.0;      //x linear movement
            this->velocities.linear.y       = 0.0;      //y linear movement
            this->mission.current_mission   = 0;
            this->mission.selected_side     = 0;
            this->e_stop_flag               = 1;
            break;
        case vehicle_user_control::KeyboardKey::KEY_e:
            if (this->status.status == 0)
            {
                this->status.status = 1;
            }
            else
            {
                this->status.status = 0;
            }
            this->velocities.angular.x      = 0.0;      //Roll
            this->velocities.angular.y      = 0.0;      //Pitch
            this->velocities.linear.x       = 0.0;      //x linear movement
            this->velocities.linear.y       = 0.0;      //y linear movement
            this->mission.current_mission   = 0;
            this->mission.selected_side     = 0;
            break;
        case vehicle_user_control::KeyboardKey::KEY_f:
            if (this->mission.selected_side == 0)
            {
                this->mission.selected_side = 1;
            }
            else
            {
                this->mission.selected_side = 0;
            }
            this->mission_debounce = 1;
            break;
    }

    if (this->status.status == 0)
    {
        switch(msg.code)
        {
            case vehicle_user_control::KeyboardKey::KEY_w:
                this->velocities.linear.x = this->default_speed;
                break;
            case vehicle_user_control::KeyboardKey::KEY_a:
                this->velocities.linear.y = -this->default_speed;
                break;
            case vehicle_user_control::KeyboardKey::KEY_s:
                this->velocities.linear.x = -this->default_speed;
                break;
            case vehicle_user_control::KeyboardKey::KEY_d:
                this->velocities.linear.y = this->default_speed;
                break;
            case vehicle_user_control::KeyboardKey::KEY_LSHIFT:
                this->velocities.linear.z = this->velocities.linear.z - 0.1;
                break;
            case vehicle_user_control::KeyboardKey::KEY_LCTRL:
                this->velocities.linear.z = this->velocities.linear.z + 0.1;
                break;
            case vehicle_user_control::KeyboardKey::KEY_RIGHT:
                this->velocities.angular.z = this->velocities.angular.z + 0.1;
                break;
            case vehicle_user_control::KeyboardKey::KEY_LEFT:
                this->velocities.angular.z = this->velocities.angular.z - 0.1;
                break;
        }

        if (abs(this->velocities.angular.z) > 3.1416)
        {
            this->velocities.angular.z = this->velocities.angular.z/abs(this->velocities.angular.z) * 
                                         (abs(this->velocities.angular.z) - 2 * 3.1416);
        }
    }
    else
    {
        switch(msg.code)
        {
            case vehicle_user_control::KeyboardKey::KEY_1:
                this->mission.current_mission = 1; // Choose Side 
                this->mission_debounce = 1;
                break;
            case vehicle_user_control::KeyboardKey::KEY_2:
                this->mission.current_mission = 2; // Buoy
                this->mission_debounce = 1;
                break;
            case vehicle_user_control::KeyboardKey::KEY_3:
                this->mission.current_mission = 3; // Torpedoes
                this->mission_debounce = 1;
                break;
            case vehicle_user_control::KeyboardKey::KEY_4:
                this->mission.current_mission = 4; // Linear Waypoint Test
                this->mission_debounce = 1;
                break;
            case vehicle_user_control::KeyboardKey::KEY_5:
                this->mission.current_mission = 5; // Orbit Waypoint Test
                this->mission_debounce = 1;
                break;
            default:
                break;
        }
    }
}

void UUVMasterNode::OnMissionFinishedReception(const std_msgs::Empty& _empty)
{
    this->mission.current_mission = 0;
}
