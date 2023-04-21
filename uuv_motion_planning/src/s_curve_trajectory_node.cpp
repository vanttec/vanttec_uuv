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

#include "vanttec_motion_planners/path_planners/include/RRT_star.hpp"
#include "vanttec_motion_planners/trajectory_planners/include/s_curve.hpp"

int main(int argc, char** argv){

    int frequency = 100;
    const float SAMPLE_TIME = 0.01;
    float MAX_TIME = 1.0;
    // bool use_map;
    int dim;
    nav_msgs::Path planned_path;
    nav_msgs::Path planned_trajectory_path;
    vanttec_msgs::Trajectory trajectory;

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
    // private_nh.param("frame_id", frame_id, std::string("map"));
    private_nh.param("frame_id", frame_id, std::string("map"));

    RRTStar path_planner(dim, SO3_bounds, MAX_TIME);
    SCurve trajectory_planner(SAMPLE_TIME);
    
    // Planned path publisher
    ros::Publisher trajectory_pub = private_nh.advertise<vanttec_msgs::Trajectory>("/uuv_motion_planning/trajectory_planner/s_curve", 1);
    ros::Publisher path_pub = private_nh.advertise<nav_msgs::Path>("/planned_path", 1);
    ros::Publisher trajectory_path_pub = private_nh.advertise<nav_msgs::Path>("/planned_trajectory_path", 1);
    ros::Subscriber map_sub = private_nh.subscribe("/map", 1, &RRTStar::save2DMap, &path_planner);

    trajectory_planner.setKinematicConstraints(X_MAX, Y_MAX, Z_MAX);

    path_planner.setStartAndGoal(start, goal);

    while(!path_planner.map_arrived_){
        ros::spinOnce();  // So map messages are received before processing solution
    }

    if(path_planner.solution_found_){
        planned_path = path_planner.getPath(frame_id);
        trajectory_planner.setPath(planned_path);
        // ROS_INFO_STREAM("Path found");
        // trajectory_planner.setStartTime(ros::Time::now().toSec());  // VERY IMPORTANT TO SET THIS JUST BEFORE ENTERING THE WHILE LOOP. WONT WORK IF NOT
        trajectory_planner.calculateTrajectory();
        planned_trajectory_path.header.frame_id = frame_id;
        trajectory = trajectory_planner.getTrajectory();

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
            // trajectory_planner.calculateTrajectory(ros::Time::now().toSec());
            path_pub.publish(planned_path);
            trajectory_pub.publish(trajectory);
            trajectory_path_pub.publish(planned_trajectory_path);
            loop_rate.sleep();
        }
    } else
        ROS_INFO_STREAM("Path not found");

    return 0;
}