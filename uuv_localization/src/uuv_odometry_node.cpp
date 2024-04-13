/** ----------------------------------------------------------------------------
 * @file: uuv_odometry_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS odometry node for the UUV. Uses uuv_odometry library.
 * -----------------------------------------------------------------------------
 **/

#include "odometry_calculator.hpp"

#include <ros/ros.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_odometry_node");
    ros::NodeHandle nh;
    
    ros::Rate           cycle_rate(int(1 / SAMPLE_TIME_S));
    OdometryCalculator  odom_calc(SAMPLE_TIME_S);
    
    ros::Publisher  uuv_pose    = nh.advertise<geometry_msgs::Pose>("/uuv_control/odometry_calculator/pose", 1000);
    ros::Publisher  uuv_twist   = nh.advertise<geometry_msgs::Twist>("/uuv_control/odometry_calculator/twist", 1000);
    ros::Publisher  uuv_accel   = nh.advertise<geometry_msgs::Accel>("/uuv_control/odometry_calculator/accel", 1000);

    ros::Subscriber uuv_linear_accel = nh.subscribe("/vectornav/ins_3d/ins_acc", 
                                                    10, 
                                                    &OdometryCalculator::AccelPubCallback, 
                                                    &odom_calc);
    ros::Subscriber uuv_angular_rate = nh.subscribe("/vectornav/ins_3d/ins_ar", 
                                                    10, 
                                                    &OdometryCalculator::AngularRateCallback, 
                                                    &odom_calc);
    ros::Subscriber uuv_angular_pose = nh.subscribe("/vectornav/ins_3d/ins_ypr", 
                                                    10, 
                                                    &OdometryCalculator::AngularPositionCallback, 
                                                    &odom_calc);
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* Update Parameters with new info */
        odom_calc.UpdateParameters();

        /* Publish Odometry */
        uuv_pose.publish(odom_calc.pose);
        uuv_twist.publish(odom_calc.twist);
        uuv_accel.publish(odom_calc.accel);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }

    return 0;
}