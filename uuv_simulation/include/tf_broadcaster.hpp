/** ----------------------------------------------------------------------------
 * @file: tf_broadcaster.hpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Used to publish the current pose of the simulated UUV and correctly
 *         represent it in RViz.
 * -----------------------------------------------------------------------------
 **/

#ifndef __TF_BROADCASTER_H__
#define __TF_BROADCASTER_H__

#include <math.h>
#include <ros/ros.h>
#include <string.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class TfBroadcaster
{
    public:
        
        nav_msgs::Path                  path;
        tf2_ros::TransformBroadcaster   br;

        std::string parent_frame;
        std::string child_frame;
        
        TfBroadcaster(const std::string& _parent, const std::string& _child);
        ~TfBroadcaster();

        void BroadcastTransform(const geometry_msgs::Pose& msg);
};

#endif