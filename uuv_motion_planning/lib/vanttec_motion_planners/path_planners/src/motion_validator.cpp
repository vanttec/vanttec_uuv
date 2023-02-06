/** ----------------------------------------------------------------------------
 * @file: motion_validator.cpp
 * @date: February 1, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Motion validator class definition.
 * -----------------------------------------------------------------------------
 **/

#include "vanttec_motion_planners/path_planners/include/motion_validator.hpp"

PlannerMotionValidator::PlannerMotionValidator(const ompl::base::SpaceInformationPtr &si)
    : ompl::base::MotionValidator(si){
    ROS_INFO_STREAM("Motion Validator Created");
    }

PlannerMotionValidator::~PlannerMotionValidator(){}

void PlannerMotionValidator::setMap(nav_msgs::OccupancyGrid& map){
    map_ = map;
    map_resolution_ = map_.info.resolution;  // m/pixel
    map_width_ = (int) map_.info.width;
    map_height_ = (int) map_.info.height;
    map_origin_ = map_.info.origin;
}

bool PlannerMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
    const ompl::base::SE3StateSpace::StateType* state_1 = s1->as<ompl::base::SE3StateSpace::StateType>();
    const ompl::base::SE3StateSpace::StateType* state_2 = s2->as<ompl::base::SE3StateSpace::StateType>();

    // Extract the path pos (x,y) from its state
    double x1 = state_1->getX();
    double y1 = state_1->getY();

    double x2 = state_2->getX();
    double y2 = state_2->getY();

    // Coordinates to index
    int i1 = (int(x1) - map_origin_.position.x) / map_resolution_;
    int j1 = (int(y1) - map_origin_.position.y) / map_resolution_;

    int i2 = (int(x2) - map_origin_.position.x) / map_resolution_;
    int j2 = (int(y2) - map_origin_.position.y) / map_resolution_;

    // Bresenhams Line Generation
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);

    // If slope is less than one
    // ROS_INFO_STREAM("Checking motion between two states");
    if (dx > dy) {
        // passing argument as 0 to plot(x,y)
        return bresenhams(x1, y1, x2, y2, dx, dy, 0);
    }
    // if slope is greater than or equal to 1
    else {
        // passing argument as 1 to plot (y,x)
        return bresenhams(y1, x1, y2, x2, dy, dx, 1);
    }
}

bool PlannerMotionValidator::bresenhams(int x1, int y1, int x2, int y2, int dx,
               int dy, int decide) const
{
    // https://www.geeksforgeeks.org/bresenhams-line-generation-algorithm/
    int pk = 2 * dy - dx;
    int grid_value = 0;

    for (int i = 0; i <= dx; i++) {
        // cout << x1 << "," << y1 << endl;
        // checking either to decrement or increment the
        // value if we have to plot from (0,100) to (100,0)
        x1 < x2 ? x1++ : x1--;
        if (pk < 0) {
            // decision value will decide to plot
            // either  x1 or y1 in x's position
            if (decide == 0) {
                // ROS_INFO_STREAM("State 1");
                grid_value = checkGridValue(x1, y1);
                pk = pk + 2 * dy;
            }
            else {
                //(y1,x1) is passed in xt
                // ROS_INFO_STREAM("State 2");
                grid_value = checkGridValue(y1, x1);
                pk = pk + 2 * dy;
            }
        }
        else {
            y1 < y2 ? y1++ : y1--;
            if (decide == 0) {
                // ROS_INFO_STREAM("State 3");
                grid_value = checkGridValue(x1, y1);
            }
            else {
                // ROS_INFO_STREAM("State 4");
                 grid_value = checkGridValue(y1, x1);
            }
            pk = pk + 2 * dy - 2 * dx;
        }
        if(grid_value != 0){ // 0 -> white (no object)
            // ROS_INFO_STREAM("Invalid cell");
            return false;
        }
    }
    // ROS_INFO_STREAM("Valid cell");
    return true;
}

int PlannerMotionValidator::checkGridValue(int x, int y) const {
    int grid_value = map_.data[x*map_width_ + y];
    // ROS_INFO_STREAM("x: " << x << ", y: " << y << ", map_width_:" << map_width_
    //                 << ", Cell: " << x*map_width_ + y << ", Grid value: " << grid_value);
    return grid_value;
}

bool PlannerMotionValidator::checkMotion(const ompl::base::State *state1,
                                    const ompl::base::State *state2,
                                    std::pair<ompl::base::State *, double> &lastValid) const {
  return checkMotion(state1, state2);
}