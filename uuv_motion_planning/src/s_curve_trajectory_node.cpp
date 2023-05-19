/** ----------------------------------------------------------------------------
 * @file: s_curve_trajectory_planner.hpp
 * @date: November 22, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 2D s-curve trajectory planner node.The path is in the NWU frame (def in rviz)
 *         as the collisions are found in this frame 
 * -----------------------------------------------------------------------------
 **/

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "vanttec_msgs/Trajectory.h"

#include "vanttec_motion_planners/path_planners/sampling_based/RRT_star.hpp"
#include "vanttec_motion_planners/trajectory_planners/s_curve/s_curve.hpp"

int main(int argc, char** argv){

    int frequency = 100;
    const float SAMPLE_TIME = 0.01;
    float MAX_TIME = 1.0;
    // bool use_map;
    int dim;
    nav_msgs::Path planned_path;
    nav_msgs::Path planned_trajectory_path;
    vanttec_msgs::Trajectory trajectory;
    vanttec_msgs::Trajectory trajectory_ned; // to send to controller

    const std::array<float, 5> X_MAX = {0, 1.08, 3.112, 7, 17.3};
    // const std::array<float, 5> Y_MAX =,{0 0.8752 0.769 0.51 1.19};
    const std::array<float, 5> Y_MAX = {0, 0.5, 0.5, 0.5, 0.5};
    const std::array<float, 5> Z_MAX = {0, 0.88, 1.96, 3.7, 21};

    std::vector<float> SO3_bounds; // Low, High
    std::vector<float> start;
    std::vector<float> goal;

    std::string frame_id;

    // Init ROS node
    ros::init(argc, argv, "trajectory_planner_node");

    // Create node handler
    ros::NodeHandle private_nh("~");

    // Setup the ROS loop rate
    ros::Rate loop_rate(frequency);

    // private_nh.getParam("use_map", use_map);
    private_nh.getParam("state_space_dimension", dim);
    private_nh.getParam("SO3_space_bounds", SO3_bounds);
    private_nh.getParam("start_point", start);
    private_nh.getParam("goal_point", goal);
    private_nh.getParam("frame_id", frame_id);

    RRTStar path_planner(dim, SO3_bounds, MAX_TIME);
    SCurve trajectory_planner(SAMPLE_TIME);
    
    // Planned path publisher
    ros::Publisher trajectory_pub = private_nh.advertise<vanttec_msgs::Trajectory>("/uuv_motion_planning/trajectory_planner/s_curve", 1);
    ros::Publisher path_pub = private_nh.advertise<nav_msgs::Path>("/planned_path", 1);
    ros::Publisher trajectory_path_pub = private_nh.advertise<nav_msgs::Path>("/planned_trajectory_path", 1);
    ros::Subscriber map_sub = private_nh.subscribe("/map", 1, &RRTStar::save2DMap, &path_planner);

    trajectory_planner.setKinematicConstraints(X_MAX, Y_MAX, Z_MAX);

    // NED TO NWU (default in rviz) to visualize in rviz.
    // Path must be calculated in this frame as collisions are found in this frame
    start[1] = -start[1];
    start[2] = -start[2];
    goal[1] = -goal[1];
    goal[2] = -goal[2];
    path_planner.setStartAndGoal(start, goal);

    while(!path_planner.map_arrived_){
        ros::spinOnce();  // So map messages are received before processing solution
    }

    if(path_planner.solution_found_){
        planned_path = path_planner.getPath(frame_id);
        trajectory_planner.setPath(planned_path); // this path is in the world frame (NWU)
        trajectory_planner.calculateTrajectory(); // this trajectory is in the world frame (NWU)
        planned_trajectory_path.header.frame_id = frame_id;
        trajectory = trajectory_planner.getTrajectory(); // in the world frame (NWU)
        trajectory_ned = trajectory_planner.getNEDTrajectory(); // in NED frame

        for(int i = 0; i < trajectory.eta_pose.size(); ++i){
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = trajectory.eta_pose[i].x;
            pose.pose.position.y = trajectory.eta_pose[i].y;
            pose.pose.position.z = trajectory.eta_pose[i].z;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.header.frame_id = frame_id;
            pose.header.stamp = ros::Time::now();
            planned_trajectory_path.poses.push_back(pose);
        }

        while (ros::ok()){
            path_pub.publish(planned_path); // to visualize in rviz
            trajectory_path_pub.publish(planned_trajectory_path); // to visualize in rviz
            trajectory_pub.publish(trajectory_ned); // for controller

            loop_rate.sleep();
        }
    } else
        ROS_ERROR_STREAM("Path not found");

    return 0;
}