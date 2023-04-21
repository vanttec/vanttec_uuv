/** ----------------------------------------------------------------------------
 * @file: s_curve.hpp
 * @date: November 16, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Lineal DOFs s-curve class definition.
 * -----------------------------------------------------------------------------
 **/

// IFNDEF ----------------------------------------------------------------------
#ifndef __S_CURVE__
#define __S_CURVE__

// INCLUDES --------------------------------------------------------------------
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>

#include "vanttec_motion_planners/path_planners/include/RRT_star.hpp"

#include "vanttec_msgs/Trajectory.h"

// CLASS DECLARATION -----------------------------------------------------------
class SCurve {
    private:
        // MEMBERS -------------------------------------------------------------
        /* Time */
        float SAMPLE_TIME_;
        double total_execution_time_;
        double start_time_;
        double T_sync_;

        // Update total time
        bool wpnts_change_;
        bool two_wpnts_;

        // Time synchronization
        float lambda_x_;
        float lambda_y_;
        float lambda_z_;

        // Kinematics limits (D Vmax Amax Jmax Smax)
        std::array<float,5> X_MAX_;
        std::array<float,5> Y_MAX_;
        std::array<float,5> Z_MAX_;
        std::array<float,5> ABS_X_MAX_;
        std::array<float,5> ABS_Y_MAX_;
        std::array<float,5> ABS_Z_MAX_;
        // std::array<float,5> Kine_MIN_;  // minimum kinematics among all the max

        // Kinematics (D V A J S)
        std::array<float,5> x_;
        std::array<float,5> prev_x_;
        std::array<float,5> y_;
        std::array<float,5> prev_y_;
        std::array<float,5> z_;
        std::array<float,5> prev_z_;

        // Time intervals
        std::array<double,5> T_x_;
        std::array<double,5> T_y_;
        std::array<double,5> T_z_;
        
        // Kinematic profiles (Jerk, Acceleration, Velocity, Position)
        std::array<float,4> K_x_;
        std::array<float,4> K_y_;
        std::array<float,4> K_z_;
        
        // Kinematic var identifier
        typedef enum {
            SURGE,
            SWAY,
            HEAVE,
            ROLL,
            PITCH,
            YAW
        } KinematicVar_;

        nav_msgs::Path path_;
        // std::vector<std::array<float, 3>> predefined_path_;
        size_t path_size_;

        vanttec_msgs::Trajectory trajectory_;

        // METHODS -------------------------------------------------------------
        // Description: calculate time intervals and kinematics
        //
        // @param var: Lineal DOF to be calculated
        // @return: none
        void calculateTimeIntervals(const KinematicVar_& var);

        // Description: calculate jerk
        //
        // @param var: Lineal DOF to be calculated
        // @return: jerk value associated
        float calculateJerk(double current_time, const KinematicVar_& var);

        // Description: update start and goal positions
        //
        // @param current_time: current time
        // @return: none
        void updatePathSegment(std::array<float,3> start, std::array<float,3> goal);

        // Description: save complete trajectory
        //
        //
        void saveTrajectory();

        // Description: apply time synchronization technique to scale all kinematics
        //
        //
        void timeSynchronization();

        // Description: calculate acceleration and velocity profiles for path segment (two waypoints)
        //
        // @param t: current time
        void calculateTrajectorySegment(double t);

    public:
        // CONSTRUCTOR AND DESTRUCTOR ------------------------------------------
        // Constructor. 
        SCurve(const float sample_time);
        // Destructor.
        ~SCurve();

        // METHODS -------------------------------------------------------------
        // Description: set start and goal positions
        //
        // @param start: start pose
        // @param goal: goal pose
        // void updatePathSegment(const std::array<float,3>& start, const std::array<float,3>& goal);

        // Description: set kinematic constraints for all degrees of freedom
        //
        // @param x_max_: x maximum kinematic constraints (Distance, Velocity, Acceleration, Jerk, Snap)
        // @param y_max_: y maximum kinematic constraints (Distance, Velocity, Acceleration, Jerk, Snap)
        // @param z_max_: z maximum kinematic constraints (Distance, Velocity, Acceleration, Jerk, Snap)
        void setKinematicConstraints(const std::array<float,5>& x_max, const std::array<float,5>& y_max, const std::array<float,5>& z_max);

        // Description: calculate acceleration and velocity profiles
        //
        //
        void calculateTrajectory();

        // Description: calculate acceleration and velocity profiles for path segment (two waypoints)
        //
        // @param start: start point
        // @param goal: goal point
        void calculateTrajectory(std::array<float,3> start, std::array<float,3> goal);

        // Description: set starting time
        //
        //
        // void setStartTime(const double start_time);

        // Description: set path
        //
        //
        void setPath(const nav_msgs::Path& path);

        // Description: return trajectory as ros msg
        //
        // @return: trajectory
        vanttec_msgs::Trajectory getTrajectory();

};

#endif