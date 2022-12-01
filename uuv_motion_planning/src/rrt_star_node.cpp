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

#include "RRT_star.hpp"

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "RRTStar");

    // Create node handler
    ros::NodeHandle node_handle("~");

    std::vector<float> SO3_bounds; // Low, High

    std::vector<float> start;
    std::vector<float> goal;

    float MAX_TIME = 1.0;
    int dim;
    bool use_map;

    // node_handle.getParam("use_map", use_map);
    node_handle.getParam("state_space_dimension", dim);
    node_handle.getParam("SO3_space_bounds", SO3_bounds);
    node_handle.getParam("planner_frame_start", start);
    node_handle.getParam("planner_frame_goal", goal);

    RRTStar planner(dim, SO3_bounds, MAX_TIME);

    // Setup the ROS loop rate
    ros::Rate loop_rate(100);

    // Planned path publisher
    ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("planned_path", 1000);

    // Occupancy map subscriber
    ros::Subscriber map_sub = node_handle.subscribe("/map", 10, &RRTStar::save2DMap, &planner);

    while (ros::ok()){
        nav_msgs::Path plannedPath;
        plannedPath = planner.planPath(start, goal);

        // Publish the planned path
        path_pub.publish(plannedPath);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}