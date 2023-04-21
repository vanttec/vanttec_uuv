/** ----------------------------------------------------------------------------
 * @file: RRT_star_2D.hpp
 * @date: November 29, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: 2D or 3D RRT* path planner class definition.
 * -----------------------------------------------------------------------------
 **/

// INCLUDES --------------------------------------------------------------------
#include "vanttec_motion_planners/path_planners/include/RRT_star.hpp"

RRTStar::RRTStar(int dim, const std::vector<float>& so3_bounds, const float max_time){
    dim_ = dim;
    max_step_length_ = 0.5;
    resolution_ = 0.1;
    MAX_PLAN_TIME_ = max_time;
    map_arrived_ = false;
    solution_found_ = false;
    collision_safety_radius_ = 0.2;

    // Set the bounds for the R^2 part of SE(3)
    bounds_ = std::make_shared<ompl::base::RealVectorBounds>(3);

    // Same bounds for x, y and z
    bounds_->setLow(so3_bounds[0]);
    bounds_->setHigh(so3_bounds[1]);

    ROS_INFO_STREAM("RRT* instance created");
    configure();
}

void RRTStar::configure(){
    // Search space
    auto space(std::make_shared<ompl::base::SE3StateSpace>());

    // Set bounds for the state space we are planning in
    space->setBounds(*bounds_);

    // Simple setup
    ss_ = std::make_shared<ompl::geometric::SimpleSetup>(space);

    ss_->setStateValidityChecker([this](const ompl::base::State *state) { return isValid(state); });

    std::shared_ptr<PlannerMotionValidator> motion_validator = std::make_shared<PlannerMotionValidator>(ss_->getSpaceInformation());
    motion_validator->setMap(map_);
    ss_->getSpaceInformation()->setMotionValidator(motion_validator);
    
    ss_->getSpaceInformation()->setStateValidityCheckingResolution(resolution_);

    ss_->setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss_->getSpaceInformation()));
    ss_->setOptimizationObjective(getPathLengthObjective());

    ROS_INFO_STREAM("Planner configured");
}

RRTStar::~RRTStar(){
    // Empty
}

// std::optional<nav_msgs::Path> RRTStar::planPath(){
void RRTStar::planPath(){
    // Start position
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(ss_->getStateSpace());
    start->setXYZ(start_point[0],start_point[1],start_point[2]);
    start->rotation().setIdentity();

    // Goal position
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(ss_->getStateSpace());
    goal->setXYZ(goal_point[0],goal_point[1],goal_point[2]);
    goal->rotation().setIdentity();

    ss_->setStartAndGoalStates(start, goal);

    // solve motion planning problem when map is available
    // nav_msgs::Path planned_path;
    // if(map_arrived_){
        ROS_INFO_STREAM("Searching solution...");
        ompl::base::PlannerStatus solved = ss_->solve(0.1);
        if (solved) {
            // Get the planned path
            ROS_INFO_STREAM("Path solution found");
            solution_found_ = true;
            // return getPath();
        } else {
            ROS_ERROR("No path solution found");
        }
    // }
    // std::cout << planned_path << std::endl;
    // return std::nullopt;
}

nav_msgs::Path RRTStar::getPath(std::string frame_id){
    nav_msgs::Path planned_path;
    planned_path.header.frame_id = frame_id;

    // Get the obtained path
    ompl::geometric::PathGeometric path = ss_->getSolutionPath();
    // Print the path to screen
    // path.print(std::cout);
    // iterate over each position
    for(unsigned int i=0; i<path.getStateCount(); ++i){
        // Get state
        const ompl::base::State* state = path.getState(i);
        // Get x coord of the robot
        const double coord_x = state->as<ompl::base::SE3StateSpace::StateType>()->getX();
        // Get y coord of the robot
        const double coord_y = state->as<ompl::base::SE3StateSpace::StateType>()->getY();
        // Get z coord of the robot
        const double coord_z = state->as<ompl::base::SE3StateSpace::StateType>()->getZ();

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = coord_x;
        pose.pose.position.y = coord_y;
        pose.pose.position.z = coord_z;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.frame_id = "/map";
        pose.header.stamp = ros::Time::now();
        planned_path.poses.push_back(pose);
    }
    return planned_path;
}

ompl::base::OptimizationObjectivePtr RRTStar::getPathLengthObjective() {
    return ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(ss_->getSpaceInformation()));
}

void RRTStar::save2DMap(const nav_msgs::OccupancyGrid::ConstPtr map){
    map_arrived_ = true;
    map_ = *map;
    map_resolution_ = map_.info.resolution;  // m/cell (cell = pixel)
    map_width_ = (int) map_.info.width;      // cells
    map_height_ = (int) map_.info.height;    // cells
    map_origin_ = map_.info.origin;

    float x_limits[2]; // Low, High
    float y_limits[2]; // Low, High

    x_limits[0] = map_origin_.position.x;
    x_limits[1] = map_width_*map_resolution_ + map_origin_.position.x;

    y_limits[0] = map_origin_.position.y;
    y_limits[1] = map_height_*map_resolution_ + map_origin_.position.y;

    // ROS_INFO_STREAM("Map Resolution: " << map_resolution_);
    // ROS_INFO_STREAM("Map origin. x = " << map_origin_.position.x << ", y =" << map_origin_.position.y);
    // ROS_INFO_STREAM("Map width (m)= " << map_width_*map_resolution_);
    // ROS_INFO_STREAM("Map height (m)= " << map_height_*map_resolution_);
    // ROS_INFO_STREAM("X coord. Low = " << x_limits[0] << " high = " << x_limits[1]);
    // ROS_INFO_STREAM("Y coord. Low = " << y_limits[0] << " high = " << y_limits[1]);

    // x bounds
    bounds_->setLow( 0, x_limits[0]);
    bounds_->setHigh(0, x_limits[1]);

    // y bounds
    bounds_->setLow( 1, y_limits[0]);
    bounds_->setHigh(1, y_limits[1]);

    // z bounds
    bounds_->setLow( 2,0);
    bounds_->setHigh(2,0);

    ROS_INFO_STREAM("Map saved");

    configure();

    planPath();
}

bool RRTStar::isValid(const ompl::base::State *state) const {
    const ompl::base::SE3StateSpace::StateType* path_state = state->as<ompl::base::SE3StateSpace::StateType>();

    // Extract the path pos (x,y) from its state
    double x = path_state->getX();
    double y = path_state->getY();

    // Coordinates to index
    int i = int((x - map_origin_.position.x) / map_resolution_);
    int j = int((y - map_origin_.position.y) / map_resolution_);

    int cell_radius = collision_safety_radius_ / map_resolution_;
    int cell;

    // ROS_INFO_STREAM("i:" << i);
    // ROS_INFO_STREAM("j:" << j);
    // ROS_INFO_STREAM("x:" << x);
    // ROS_INFO_STREAM("y:" << y);
    // ROS_INFO_STREAM("Map resolution: " << map_resolution_);
    // ROS_INFO_STREAM("cell radius: " << cell_radius);
    // ROS_INFO_STREAM("Map width (cells)= " << map_width_);
    // ROS_INFO_STREAM("Map height (cells)= " << map_height_);
    // ROS_INFO_STREAM("Map origin x:" << map_origin_.position.x);
    // ROS_INFO_STREAM("Map origin y:" << map_origin_.position.y);
    // ROS_INFO_STREAM("Map X coord (m). Low = " << bounds_->low[0] << " high = " << bounds_->high[0]);
    // ROS_INFO_STREAM("Map Y coord (m). Low = " << bounds_->low[1] << " high = " << bounds_->high[1]);

    // Iterate over the state surrounding cells to check for obstacles
    for(int m = i-cell_radius; m <= i+cell_radius; m++){
        for(int n = j-cell_radius; n <= j+cell_radius; n++){
            if(m > 0 && m < map_width_){  
                if(n > 0 && n < map_height_){
                    // ROS_INFO_STREAM("m:" << m);
                    // ROS_INFO_STREAM("n:" << n);
                    // ROS_INFO_STREAM("cell #: " << m*map_width_ + n);
                    cell = map_.data[m*map_width_ + n];
                    // ROS_INFO_STREAM("Cell value: " << cell);
                    if (cell != 0)
                        return false;
                    else
                        break;
                }
            }
        }
        if(cell == 0)
            break;
    }
    return true;
}

// bool RRTStar::checkMotion(const State *s1, const State *s2) {
    
// }

// double RRTstar::clearance(const ompl::base::State* state) const {
//     const ompl::base::SE3StateSpace::StateType* path_state = state->as<ompl::base::SE3StateSpace::StateType>();

//     // Extract the path pos (x,y) from its state
//     double x = path_state->values[0];
//     double y = path_state->values[1];

//     return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
// }

void RRTStar::setStartAndGoal(const std::vector<float>& start_point, const std::vector<float>& goal_point){
    this->start_point = start_point;
    this->goal_point = goal_point;
}
