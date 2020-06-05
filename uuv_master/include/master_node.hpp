#ifndef __UUV_MASTER_NODE_H__
#define __UUV_MASTER_NODE_H__

#include "uuv_master/UUVMasterStatus.h"
#include "vehicle_user_control/KeyboardKey.h"

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>


class UUVMasterNode
{
    public:
        
        std_msgs::Empty                 emergency_stop;
        geometry_msgs::Twist            velocities;
        uuv_master::UUVMasterStatus     status;

        float default_speed;
        int e_stop_flag;

        UUVMasterNode(const float default_speed);
        ~UUVMasterNode();

        void keyboardUpCallback(const vehicle_user_control::KeyboardKey& msg);
        void keyboardDownCallback(const vehicle_user_control::KeyboardKey& msg);
};

#endif // __UUV_MASTER_NODE_H__