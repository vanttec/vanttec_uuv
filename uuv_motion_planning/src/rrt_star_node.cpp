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

#include "../vanttec_motion_planners/path_planners/rrt_star/include/RRT_star_2D.hpp"

nav_msgs::OccupancyGrid global_map;

// occupancy map callback
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg) {
    global_map = *mapMsg;
}


int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "RRTStar2D");

    // Create node handler
    ros::NodeHandle node_handle("~");

    std::array<float, 2> x_limits = {-1.0, 1.0}; // Low, High
    std::array<float, 2> y_limits = {-1.0, 1.0}; // Low, High

    std::array<float, 3> start = {-1.0, -1.0, -0.5};
    std::array<float, 3> goal =  { 1.0,  1.0,  0.5};

    RRTStar2D planner(node_handle, x_limits, y_limits, 1.0);

    // Setup the ROS loop rate
    ros::Rate loop_rate(100);

    // Planned path publisher
    ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("planned_path", 1000);

    // Occupancy map subscriber
    ros::Subscriber map_sub = node_handle.subscribe("/map", 10, mapCallback);

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