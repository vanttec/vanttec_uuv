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

#include "vanttec_msgs/Trajectory.h"

// CLASS DECLARATION -----------------------------------------------------------
class SCurve {
    private:
        // MEMBERS -------------------------------------------------------------
        float SAMPLE_TIME_;

        /* Start time */
        double start_time_;

        // Kinematics limits (D Vmax Amax Jmax Smax)
        std::array<float,5> X_MAX_;
        std::array<float,5> Y_MAX_;
        std::array<float,5> Z_MAX_;
        std::array<float,5> Kine_MIN_;  // minimum kinematics among all the max

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

        // Starting point
        std::array<float,3> start_;

        // Goal point
        std::array<float,3> goal_;

        // Kinematic var identifier
        typedef enum {
            SURGE,
            SWAY,
            HEAVE,
            ROLL,
            PITCH,
            YAW
        } KinematicVar_;

        // METHODS -------------------------------------------------------------
        // Description: calculate time intervals and kinematics
        //
        // @param var: Lineal DOF to be calculated
        void calculateTimeIntervals(const KinematicVar_& var);

        // Description: calculate jerk
        //
        // @param var: Lineal DOF to be calculated
        // @return: jerk value associated
        float calculateJerk(double current_time, const KinematicVar_& var);

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
        void setStartAndGoal(const std::array<float,3>& start, const std::array<float,3>& goal);

        // Description: set kinematic constraints for all degrees of freedom
        //
        // @param x_max_: x maximum kinematic constraints (Distance, Velocity, Acceleration, Jerk, Snap)
        // @param y_max_: y maximum kinematic constraints (Distance, Velocity, Acceleration, Jerk, Snap)
        // @param z_max_: z maximum kinematic constraints (Distance, Velocity, Acceleration, Jerk, Snap)
        void setKinematicConstraints(const std::array<float,5>& x_max, const std::array<float,5>& y_max, const std::array<float,5>& z_max);
        // void velocitySynchronization();

        // Description: calculate acceleration and velocity profiles
        //
        //
        void calculateTrajectory(const double current_time);

        // Description: set starting time
        //
        //
        void setStartTime(const double start_time);

        // Description: return trajectory as ros msg
        //
        // @return: trajectory
        vanttec_msgs::Trajectory getTrajectory();
};

#endif