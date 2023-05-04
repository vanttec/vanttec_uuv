/** ----------------------------------------------------------------------------
 * @file: master_node.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Master node, in charge of receiving control input and managing other
 *         nodes in the architecture.
 * -----------------------------------------------------------------------------
 **/

#ifndef __UUV_MASTER_NODE_H__
#define __UUV_MASTER_NODE_H__

#include "vanttec_uuv/MasterStatus.h"
#include "vanttec_uuv/MissionStatus.h"
#include "vehicle_user_control/KeyboardKey.h"

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class UUVMasterNode
{
    public:
        
        std_msgs::Empty                 emergency_stop;
        geometry_msgs::Twist            velocities;
        vanttec_uuv::MasterStatus       status;
        vanttec_uuv::MissionStatus      mission;

        float default_speed;
        int e_stop_flag;
        int mission_debounce;

        UUVMasterNode(const float default_speed);
        ~UUVMasterNode();

        void OnMissionFinishedReception(const std_msgs::Empty& _empty);
        void keyboardUpCallback(const vehicle_user_control::KeyboardKey& msg);
        void keyboardDownCallback(const vehicle_user_control::KeyboardKey& msg);
};

#endif // __UUV_MASTER_NODE_H__