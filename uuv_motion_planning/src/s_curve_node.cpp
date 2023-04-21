/** ----------------------------------------------------------------------------
 * @file: s_curve_trajectory_planner.hpp
 * @date: November 22, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 2D s-curve trajectory planner node.
 * -----------------------------------------------------------------------------
 **/

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "vanttec_msgs/Trajectory.h"

#include "vanttec_motion_planners/trajectory_planners/include/s_curve.hpp"

int main(int argc, char** argv){
    int frequency = 100;
    // Init ROS node
    ros::init(argc, argv, "s_curve_node");

    // Create node handler
    ros::NodeHandle node_handle("~");

    const float SAMPLE_TIME = 0.01;
    SCurve trajectory_planner(SAMPLE_TIME);

    // std::array<float, 3> start = {-1, -2, -1};
    // std::array<float, 3> goal =  { 1.05489,  1.10891, 1.0};

    std::array<float, 3> start = {0, 0, 0};
    std::array<float, 3> goal =  {1.5, 1.5, 1.5};

    const std::array<float, 5> X_MAX = {0, 1.08, 3.112, 7, 17.3};
    // const std::array<float, 5> Y_MAX =,{0 0.8752 0.769 0.51 1.19};
    const std::array<float, 5> Y_MAX = {0, 0.5, 0.5, 0.5, 0.5};
    const std::array<float, 5> Z_MAX = {0, 0.88, 1.96, 3.7, 21};
    
    trajectory_planner.setKinematicConstraints(X_MAX, Y_MAX, Z_MAX);

    // Setup the ROS loop rate
    ros::Rate loop_rate(frequency);

    // Planned path publisher
    ros::Publisher trajectory_pub = node_handle.advertise<vanttec_msgs::Trajectory>("/uuv_motion_planning/trajectory_planner/s_curve", 1);

    // trajectory_planner.setStartTime(ros::Time::now().toSec());
    trajectory_planner.calculateTrajectory(start, goal);
    while (ros::ok()){
        vanttec_msgs::Trajectory trajectory;
        // ROS_INFO_STREAM(ros::Time::now());
        // trajectory_planner.calculateTrajectory(ros::Time::now().toSec());
        trajectory = trajectory_planner.getTrajectory();

        trajectory_pub.publish(trajectory);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}