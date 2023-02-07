/** ----------------------------------------------------------------------------
 * @file: motion_validator.hpp
 * @date: February 1, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Motion validator class definition.
 * -----------------------------------------------------------------------------
 **/

// IFNDEF ----------------------------------------------------------------------
#ifndef __MOTION_VALIDATOR__
#define __MOTION_VALIDATOR__

// INCLUDES --------------------------------------------------------------------
#include <ompl/base/MotionValidator.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <ros/console.h>

// CLASS DECLARATION -----------------------------------------------------------
class PlannerMotionValidator : public ompl::base::MotionValidator{
    private:
        // MEMBERS -------------------------------------------------------------
        nav_msgs::OccupancyGrid map_;
        geometry_msgs::Pose map_origin_;
        float map_resolution_;
        int map_width_;
        int map_height_;
        bool map_arrived_;

    public:
        // CONSTRUCTOR AND DESTRUCTOR ------------------------------------------
        // Constructor. 
        PlannerMotionValidator(const ompl::base::SpaceInformationPtr &si);
        // Destructor.
        ~PlannerMotionValidator();

        // METHODS -------------------------------------------------------------

        void setMap(nav_msgs::OccupancyGrid& map);
        
        // Description: checking the validity of motions - path segments between states
        //
        // @param s1: start state (assumed to be valid)
        // @param s2: goal state
        // @retval path segmente validation
        bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

        // Description: checking the validity of motions - path segments between states.
        //              MotionValidator header expects two checkMotion funct. definitions.
        //
        // @param s1: start state (assumed to be valid)
        // @param s2: goal state
        // @retval path segmente validation
        bool checkMotion(const ompl::base::State *s1,
                        const ompl::base::State *s2,
                        std::pair<ompl::base::State *, double> &lastValid) const override;

        // Description: implements Bresenham Line Algorithm to check grids under the line
        // formed by two points
        // https://www.geeksforgeeks.org/bresenhams-line-generation-algorithm/
        //
        // @param x1: first state x coordinate
        // @param y1: first state y coordinate
        // @param x2: second state x coordinate
        // @param y2: second state y coordinate
        // @param dx: x2 - x1
        // @param dy: y2 - y1
        // @param decide: 0 if dx > dy, 1 otherwise
        // @retval: true if there are only white grids
        bool bresenhams(int x1, int y1, int x2, int y2, int dx, int dy, int decide) const;

        // Description: implements Discrete Differential Analyzer to check grids under the line
        // formed by two points. This covers all the cells under the line.
        // http://eugen.dedu.free.fr/projects/bresenham/
        //
        // @param x1: first state x coordinate
        // @param y1: first state y coordinate
        // @param x2: second state x coordinate
        // @param y2: second state y coordinate
        // @retval: true if there are only white grids
        bool DDA(int x1, int y1, int x2, int y2) const;

        // Description: checks grid value from map
        //
        // @param x: state x coordinate
        // @param y: state y coordinate
        // @retval: 0(white), -1(undefine), 100(black)
        int checkGridValue(int x, int y) const;
};

#endif