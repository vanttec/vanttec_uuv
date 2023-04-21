/** ----------------------------------------------------------------------------
 * @file: RRT_star_2D.hpp
 * @date: November 29, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 2D or 3D RRT* path planner class definition.
 * -----------------------------------------------------------------------------
 **/

// IFNDEF ----------------------------------------------------------------------
#ifndef __RRT_STAR__
#define __RRT_STAR__

// INCLUDES --------------------------------------------------------------------
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include "motion_validator.hpp"

// CLASS DECLARATION -----------------------------------------------------------
class RRTStar /*: public base::MotionValidator*/{
    private:
        // MEMBERS -------------------------------------------------------------

        /* Planning space*/
        int dim_;                           // Problem dimension
        float max_step_length_;             // Max step length for planner
        float resolution_;                  // Collision resolution
        float MAX_PLAN_TIME_;
        ompl::geometric::SimpleSetupPtr ss_; // Planner simple setup
        std::shared_ptr<ompl::base::RealVectorBounds> bounds_; // Dimension bounds
        
        std::shared_ptr<ompl::base::SpaceInformation> si_;     // Planner space information

        std::vector<float> start_point;
        std::vector<float> goal_point;

        /* Environment map */
        nav_msgs::OccupancyGrid map_;
        geometry_msgs::Pose map_origin_;
        float map_resolution_;
        int map_width_;
        int map_height_;

        /* Collision avoidance */
        float collision_safety_radius_;
        
        // METHODS -------------------------------------------------------------

        // Description: set node configuration.
        //
        // No parameters
        void configure();

    public:
        bool map_arrived_;
        bool solution_found_;

        // CONSTRUCTOR AND DESTRUCTOR ------------------------------------------
        // Constructor. 
        RRTStar(int dim, const std::vector<float>& so3_bounds, const float max_time);
        // Destructor.
        ~RRTStar();

        // METHODS -------------------------------------------------------------

        // Description: plan 2D path.
        //
        // @retval: void
        // std::optional<nav_msgs::Path> planPath();
        void planPath();

        // Description: return computed path.
        //
        // @retval: planned path
        nav_msgs::Path getPath(std::string frame_id);

        // Description: get path length optimization objective for RRT*
        //
        // @retval: optimization objective
        ompl::base::OptimizationObjectivePtr getPathLengthObjective();

        // Description: plan start and goal states for planner.
        //
        // @param start: start pose
        // @param goal: goal pose
        // @retval: void
        void setStartAndGoal(const std::vector<float>& start_point, const std::vector<float>& goal_point);

        // Description: callback to save a map and configure planner
        //
        // @param map: environment map
        void save2DMap(const nav_msgs::OccupancyGrid::ConstPtr map);
        
        // Description: check if the current state is valid
        //
        // @param state: path state
        // @retval: state validation
        bool isValid(const ompl::base::State *state) const;
        
        // double clearance(const ompl::base::State* state) const;
};

#endif