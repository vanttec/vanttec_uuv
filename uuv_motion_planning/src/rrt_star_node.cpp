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
    ros::NodeHandle private_nh("~");

    std::vector<float> SO3_bounds; // Low, High
    std::vector<float> start;
    std::vector<float> goal;

    std::string frame_id;

    // private_nh.getParam("use_map", use_map);
    private_nh.getParam("state_space_dimension", dim);
    private_nh.getParam("SO3_space_bounds", SO3_bounds);
    private_nh.getParam("start_point", start);
    private_nh.getParam("goal_point", goal);
    private_nh.param("frame_id", frame_id, std::string("map"));
    
    RRTStar planner(dim, SO3_bounds, MAX_TIME);

    // Setup the ROS loop rate
    ros::Rate loop_rate(frequency);

    // Planned path publisher
    ros::Publisher path_pub = private_nh.advertise<nav_msgs::Path>("/planned_path", 1);

    // Occupancy map subscriber
    ros::Subscriber map_sub = private_nh.subscribe("/map", 1, &RRTStar::save2DMap, &planner);

    // while (ros::ok()){
    nav_msgs::Path planned_path;

    planner.setStartAndGoal(start, goal);
    while(ros::ok()){
        ros::spinOnce(); // So map messages are received before processing solution
        if(planner.solution_found_){
            planned_path = planner.getPath(frame_id);
            // Publish the planned path
            // if(planned_path != std::nullopt)
            path_pub.publish(planned_path);
        }
        loop_rate.sleep();
    }

    return 0;
}