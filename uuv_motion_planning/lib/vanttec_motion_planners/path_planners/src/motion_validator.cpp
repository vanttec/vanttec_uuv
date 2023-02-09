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
    map_resolution_ = map_.info.resolution; // m/cell (cell = pixel)
    map_width_ = (int) map_.info.width;     // cells
    map_height_ = (int) map_.info.height;   // cells
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
    int i1 = int((x1 - map_origin_.position.x) / map_resolution_);
    int j1 = int((y1 - map_origin_.position.y) / map_resolution_);

    int i2 = int((x2 - map_origin_.position.x) / map_resolution_);
    int j2 = int((y2 - map_origin_.position.y) / map_resolution_);

    // // Bresenhams Line Generation
    // int dx = abs(i2 - i1);
    // int dy = abs(j2 - j1);

    // // If slope is less than one
    // // ROS_INFO_STREAM("Checking motion between two states");
    // if (dx > dy) {
    //     // passing argument as 0 to plot(x,y)
    //     return bresenhams(i1, j1, i2, j2, dx, dy, 0);
    // }
    // // if slope is greater than or equal to 1
    // else {
    //     // passing argument as 1 to plot (y,x)
    //     return bresenhams(j1, i1, j2, i2, dy, dx, 1);
    // }
    return raytrace(i1, j1, i2, j2);
}

bool PlannerMotionValidator::raytrace(int x0, int y0, int x1, int y1) const {
    int grid_value = 0;
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int x = x0;
    int y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
        grid_value = checkGridValue(y,x); // y is x, and x is y. That is why they are interchanged
        if(grid_value != 0)
            return false;

        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else {
            y += y_inc;
            error += dx;
        }
    }
    return true;
}

bool PlannerMotionValidator::DDA(int x1, int y1, int x2, int y2) const {
    // There is a bug in straigh lines (slope = 0 or infinite)
    int grid_value = 0;

    int i;                  // loop counter
    int y_step, x_step;     // the step on y and x axis
    int error;              // the error accumulated during the increment
    int error_prev;         // vision the previous value of the error variable
    int y = y1;
    int x = x1;
    int ddy, ddx;        // compulsory variables: the double values of dy and dx
    int dx = x2 - x1;
    int dy = y2 - y1;

    if (dy < 0){
        y_step = -1;
        dy = -dy;
    } else
        y_step = 1;

    if (dx < 0) {
        x_step = -1;
        dx = -dx;
    } else
        x_step = 1;

    ddy = 2 * dy;  // work with double values for full precision
    ddx = 2 * dx;

    if (ddx >= ddy) {  // first octant (0 <= slope <= 1)
        // compulsory initialization (even for error_prev, needed when dx==dy)
        error_prev = error = dx;  // start in the middle of the square
        for (i=0 ; i < dx ; i++) {  // do not use the first point (already done)
            x += x_step;
            error += ddy;
            if (error > ddx) {  // increment y if AFTER the middle ( > )
                y += y_step;
                error -= ddx;
                // three cases (octant == right->right-top for directions below):
                if (error + error_prev < ddx)  // bottom square also
                    grid_value = checkGridValue(y-y_step, x);
                else if (error + error_prev > ddx)  // left square also
                    grid_value = checkGridValue(y, x-x_step);
                else {  // corner: bottom and left squares also
                    grid_value = checkGridValue(y-y_step, x)    ;
                    if(grid_value != 0)
                        return false;
                    grid_value = checkGridValue(y, x-x_step);
                }
                if(grid_value != 0)
                    return false;
            }
            error_prev = error;
        }
    } else {  // the same as above
        error_prev = error = dy;
        for (i=0 ; i < dy ; i++) {
            y += y_step;
            error += ddx;
            if (error > ddy) {
                x += x_step;
                error -= ddy;
                if (error + error_prev < ddy)
                    grid_value = checkGridValue(y, x-x_step);
                else if (error + error_prev > ddy)
                    grid_value = checkGridValue(y-y_step, x);
                else {
                    grid_value = checkGridValue(y-y_step, x);
                    if(grid_value != 0)
                        return false;
                    grid_value = checkGridValue(y, x-x_step);
                }
                if(grid_value != 0)
                    return false;
            }
        error_prev = error;
        }
    }
    // // assert ((y == y2) && (x == x2));  // the last point (y2,x2) has to be the same with the last point of the algorithm

    return true;
}

bool PlannerMotionValidator::bresenhams(int x1, int y1, int x2, int y2, int dx,
               int dy, int decide) const {
    int pk = 2 * dy - dx;
    int grid_value = 0;

    for (int i = 0; i <= dx; i++) {
        // cout << x1 << "," << y1 << endl;
        // checking either to decrement or increment the value
        x1 < x2 ? x1++ : x1--;
        if (pk < 0) {
            // decision value will decide to plot
            // either  x1 or y1 in x's position
            if (decide == 0) {
                // ROS_INFO_STREAM("State 1");
                grid_value = checkGridValue(x1, y1);
            }
            else {
                //(y1,x1) is passed in xt
                // ROS_INFO_STREAM("State 2");
                grid_value = checkGridValue(y1, x1);
            }
            pk += 2 * dy;
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
            pk += 2 * dy - 2 * dx;
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
    // ROS_INFO_STREAM("x: " << x << ", y: " << y << ", map_width_:" << map_width_
                    // << ", Cell: " << x*map_width_ + y);
    int grid_value = map_.data[x*map_width_ + y];
    // ROS_INFO_STREAM("Grid value: " << grid_value);
    return grid_value;
}

bool PlannerMotionValidator::checkMotion(const ompl::base::State *state1,
                                    const ompl::base::State *state2,
                                    std::pair<ompl::base::State *, double> &lastValid) const {
  return checkMotion(state1, state2);
}