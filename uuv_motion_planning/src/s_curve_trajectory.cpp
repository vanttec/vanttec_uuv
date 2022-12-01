/** ----------------------------------------------------------------------------
 * @file: s_curve_trajectory.hpp
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

#include "s_curve.hpp"

int main(int argc, char** argv){
    // Init ROS node
    ros::init(argc, argv, "s_curve_node");

    // Create node handler
    ros::NodeHandle node_handle("~");

    const float SAMPLE_TIME = 0.01;
    SCurveLinealDOF trajectory_planner(SAMPLE_TIME);

    std::array<float, 3> start = {-1.0, -1.0, -0.5};
    std::array<float, 3> goal =  { 1.0,  1.0,  0.5};

    const std::array<float, 5> X_MAX = {0 1.08 3.112 7 17.3};
    // const std::array<float, 5> Y_MAX = {0 0.8752 0.769 0.51 1.19};
    const std::array<float, 5> Y_MAX = {0 0.5 0.5 0.5 0.5};
    const std::array<float, 5> Z_MAX = {0 0.88 1.96 3.7 21};
    
    trajectory_planner.setStartAndGoal(start, goal);
    trajectory_planner.setKinematicConstraints(X_MAX, Y_MAX, Z_MAX);

    // Setup the ROS loop rate
    ros::Rate loop_rate(100);

    // Planned path publisher
    ros::Publisher trajectory_pub = node_handle.advertise<vanttec_msgs::Trajectory>("/uuv_motion_planning/trajectory_planner/s_curve", 1);

    while (ros::ok()){
        vanttec_msgs::Trajectory trajectory;

        trajectory_planner.calculateTrajectory();
        trajectory = trajectory_planner.getTrajectory();

        trajectory_pub.pub(trajectory);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}