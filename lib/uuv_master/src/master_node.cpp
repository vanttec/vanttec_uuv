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
    this->default_speed = _default_speed;
    this->e_stop_flag   = 0;
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
            this->status.desired_routine    = 0;
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
            this->status.desired_routine    = 0;
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
    }
    else
    {
        switch(msg.code)
        {
            case vehicle_user_control::KeyboardKey::KEY_1:
                this->status.desired_routine = 1;
                break;
            case vehicle_user_control::KeyboardKey::KEY_2:
                this->status.desired_routine = 2;
                break;
            case vehicle_user_control::KeyboardKey::KEY_3:
                this->status.desired_routine = 3;
                break;
            case vehicle_user_control::KeyboardKey::KEY_4:
                this->status.desired_routine = 4;
                break;
            case vehicle_user_control::KeyboardKey::KEY_5:
                this->status.desired_routine = 5;
                break;
        }
    }
}
