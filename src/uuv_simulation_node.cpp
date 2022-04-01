/** ----------------------------------------------------------------------------
 * @file: uuv_simulation_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS simulation node for the UUV. Uses uuv_simulation library.
 * -----------------------------------------------------------------------------
 **/

#include "vtec_u4_6dof_dynamic_model.hpp"
#include "EtaPose.h"

#include <ros/ros.h>
#include <stdio.h>

static const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_simulation_node");
    ros::NodeHandle nh;
        
    ros::Rate               cycle_rate(int(1 / SAMPLE_TIME_S));
    VTecU4DynamicModel      uuv_model(SAMPLE_TIME_S);
    
    ros::Publisher  uuv_accel  = nh.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_acc", 1000);
    // ros::Publisher  uuv_arate  = nh.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_ar", 1000);
    // ros::Publisher  uuv_apos   = nh.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_ypr", 1000);
    ros::Publisher  uuv_vel    = nh.advertise<geometry_msgs::Twist>("/uuv_simulation/dynamic_model/vel", 1000);
    ros::Publisher  uuv_pose   = nh.advertise<vanttec_uuv::EtaPose>("/uuv_simulation/dynamic_model/pose", 1000);

    ros::Subscriber uuv_thrust_input = nh.subscribe("/uuv_control/uuv_control_node/thrust", 
                                                    10, 
                                                    &UUVDynamic4DOFModel::ThrustCallback, 
                                                    &uuv_model);
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* Calculate Model States */
        VTecU4DynamicModel.CalculateStates();

        /* Publish Odometry */
        uuv_accel.publish(VTecU4DynamicModel.linear_acceleration);
        // uuv_arate.publish(VTecU4DynamicModel.angular_rate);
        // uuv_apos.publish(VTecU4DynamicModel.angular_position);
        uuv_vel.publish(VTecU4DynamicModel.velocities);
        uuv_pose.publish(VTecU4DynamicModel.eta_pose);
        
        /* Sleep for 10ms */
        cycle_rate.sleep();
    }

    return 0;
}