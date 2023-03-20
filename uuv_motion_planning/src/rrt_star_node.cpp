/** ----------------------------------------------------------------------------
 * @file: rrt_star_node.hpp
 * @date: November 11, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 2D RRT* path planner node.
 * -----------------------------------------------------------------------------
 **/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> 
#include <nav_msgs/OccupancyGrid.h>

#include "vanttec_motion_planners/path_planners/include/RRT_star.hpp"

int main(int argc, char** argv)
{
    float MAX_TIME = 1.0;
    // bool use_map;
    int frequency = 100;
    int dim;

    // Init ROS node
    ros::init(argc, argv, "RRTStar");

    // Create node handler
    ros::NodeHandle node_handle("~");

    std::vector<float> SO3_bounds; // Low, High
    std::vector<float> start;
    std::vector<float> goal;

    // node_handle.getParam("use_map", use_map);
    node_handle.getParam("state_space_dimension", dim);
    node_handle.getParam("SO3_space_bounds", SO3_bounds);
    node_handle.getParam("start_point", start);
    node_handle.getParam("goal_point", goal);

    RRTStar planner(dim, SO3_bounds, MAX_TIME);

    // Setup the ROS loop rate
    ros::Rate loop_rate(frequency);

    // Planned path publisher
    ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("/planned_path", 1);

    // Occupancy map subscriber
    ros::Subscriber map_sub = node_handle.subscribe("/map", 1, &RRTStar::save2DMap, &planner);

    // while (ros::ok()){
    nav_msgs::Path planned_path;

    planner.setStartAndGoal(start, goal);
    while(ros::ok()){
        ros::spinOnce(); // So map messages are received before processing solution
        if(planner.solution_found_){
            planned_path = planner.getPath();
            // Publish the planned path
            // if(planned_path != std::nullopt)
            path_pub.publish(planned_path);
        }
        loop_rate.sleep();
    }

    return 0;
}