#include <cmath>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <vanttec_uuv/GuidanceWaypoints.h>
#include <vanttec_uuv/MasterStatus.h>

#include "asmc_guidance_4dof.hpp"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S = 0.05;

geometry_msgs::Pose pose;

void pose_callback(const geometry_msgs::Pose& _pose)
{
    pose.position.x = _pose.position.x;
    pose.position.y = _pose.position.y;
    pose.position.z = _pose.position.z;
    pose.orientation.z = _pose.orientation.z;
}

int main(int argc, char **argv)
{
    ASMC_GUIDANCE_4DOF guidance_law((double) SAMPLE_TIME_S, 0.001, 0.01, 0.01, 0.01, 0.1);
    guidance_law.asmc_guidance_yaw.Kalpha = 0.1;

    geometry_msgs::Twist desired_velocities;

    float setpoints[4];
    float distance = 0;
    ros::init(argc, argv, "uuv_guidance_node");
    ros::NodeHandle nh;
    ros::Rate               cycle_rate(int(1 / SAMPLE_TIME_S));

    ros::Publisher  uuv_desired_setpoints       = nh.advertise<geometry_msgs::Twist>("/uuv_control/uuv_control_node/setpoint", 1000);

    ros::Subscriber uuv_pose                    = nh.subscribe("/uuv_simulation/dynamic_model/pose",
                                                                1000,
                                                                pose_callback);

    // ros::Subscriber uuv_waypoints               = nh.subscribe("/uuv_guidance/guidance_controller/waypoints",
    //                                                             1000,
    //                                                             &GuidanceController::OnWaypointReception,
    //                                                             &guidance_controller);

    uint32_t counter = 0;
    setpoints[0] = 5;
    setpoints[1] = 5;
    setpoints[2] = 10;
    setpoints[3] = 0;//std::atan2(setpoints[1],setpoints[0]);

    distance = std::sqrt( std::pow(setpoints[0] - pose.position.x,2) + std::pow(setpoints[1] - pose.position.y,2) );
    while(ros::ok())
    {
        guidance_law.SetSetpoints(setpoints);

        /* Run Queued Callbacks */
        ros::spinOnce();
        guidance_law.CalculateManipulation(pose);

        desired_velocities.linear.x = guidance_law.U(0);
        desired_velocities.linear.y = guidance_law.U(1);
        desired_velocities.linear.z = guidance_law.U(2);
        desired_velocities.angular.z = guidance_law.U(3);

        std::cout<<guidance_law.U(0)<<std::endl;
        std::cout<<guidance_law.U(1)<<std::endl;
        std::cout<<guidance_law.U(2)<<std::endl;
        std::cout<<guidance_law.U(3)<<std::endl;

        /* Publish Odometry */
        uuv_desired_setpoints.publish(desired_velocities);

        if(setpoints[3] < 3)
        {
            setpoints[3] += 0.1;
        }
        /* Slee for 10ms */
        cycle_rate.sleep();
    }

    return 0;
}