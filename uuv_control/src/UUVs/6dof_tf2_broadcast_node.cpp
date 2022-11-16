/** ----------------------------------------------------------------------------
 * @file: 6dof_tf2_broadcast_node.cpp
 * @date: April 10, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ROS tf broadcast node for the UUV. Uses uuv_simulation library.
 * -----------------------------------------------------------------------------
 **/

#include "6dof_tf2_broadcaster.hpp"

#include <ros/ros.h>
#include <string.h>

static const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    std::string parent_frame;
    std::string child_frame;

    ros::init(argc, argv, "tf2_broadcaster_node");
    ros::NodeHandle nh("~");
    
    nh.param<std::string>("parent_frame", parent_frame, "world");
    nh.param<std::string>("child_frame", child_frame, "uuv");

    // ROS_INFO_STREAM("Parent frame: " << parent_frame);
    // ROS_INFO_STREAM("Child frame: " << child_frame);

    ros::Rate               cycle_rate(int(1 / SAMPLE_TIME_S));
    TF2Broadcaster           tf_broadcaster(parent_frame, child_frame);
    
    ros::Publisher  uuv_path    = nh.advertise<nav_msgs::Path>("/uuv_simulation/uuv_tf_broadcast/uuv_path", 1000);
    
    ros::Subscriber uuv_pose    = nh.subscribe("/uuv_simulation/dynamic_model/eta_pose_", 
                                               10, 
                                               &TF2Broadcaster::BroadcastTransform, 
                                               &tf_broadcaster);
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* Publish Path */
        uuv_path.publish(tf_broadcaster.path);
        
        /* Sleep for 10ms */
        cycle_rate.sleep();
    }

    return 0;
}