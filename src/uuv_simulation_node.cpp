/** ----------------------------------------------------------------------------
 * @file: uuv_simulation_node.cpp
 * @date: April 10, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebas Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ROS simulation node for the UUV. Uses uuv_simulation library.
 * https://answers.ros.org/question/190920/how-can-i-subscribe-to-a-topic-using-a-parent-class-function-as-the-callback/
 * -----------------------------------------------------------------------------
 **/

#include "vtec_u4_6dof_dynamic_model.hpp"
#include "vanttec_uuv/EtaPose.h"

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
                                                    &Generic6DOFUUVDynamicModel::ThrustCallback,
                                                    dynamic_cast<Generic6DOFUUVDynamicModel*> (&uuv_model));
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* Calculate Model States */
        uuv_model.CalculateStates();

        /* Publish Odometry */
        uuv_accel.publish(uuv_model.accelerations);
        // uuv_arate.publish(uuv_model.angular_rate);
        // uuv_apos.publish(uuv_model.angular_position);
        uuv_vel.publish(uuv_model.velocities);
        uuv_pose.publish(uuv_model.eta_pose);
        
        /* Sleep for 10ms */
        cycle_rate.sleep();
    }

    return 0;
}