/** ----------------------------------------------------------------------------
 * @file: uuv_tf_broadcast_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS tf broadcast node for the UUV. Uses uuv_simulation library.
 * -----------------------------------------------------------------------------
 **/

#include "tf_broadcaster.hpp"

#include <ros/ros.h>
#include <string.h>

static const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_tf_broadcast_node");
    ros::NodeHandle nh;
        
    ros::Rate               cycle_rate(int(1 / SAMPLE_TIME_S));
    TfBroadcaster           tf_broadcaster("world", "uuv");
    
    ros::Publisher  uuv_path    = nh.advertise<nav_msgs::Path>("/uuv_simulation/uuv_tf_broadcast/uuv_path", 1000);
    
    ros::Subscriber uuv_pose    = nh.subscribe("/uuv_simulation/dynamic_model/pose", 
                                               10, 
                                               &TfBroadcaster::BroadcastTransform, 
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