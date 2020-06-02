#ifndef __UUV_4DOF_CONTROLLER_H__
#define __UUV_4DOF_CONTROLLER_H__

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class UUV4DOFController
{
    geometry_msgs::Pose     local_pose;
    geometry_msgs::Twist    local_twist;
};

#endif