/** ----------------------------------------------------------------------------
 * @file: s_curve.cpp
 * @date: November 16, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Single DOF s-curve class definition.
 * -----------------------------------------------------------------------------
 **/

// INCLUDES --------------------------------------------------------------------
#include "vanttec_motion_planners/trajectory_planners/include/s_curve.hpp"

// CONSTRUCTOR ------------------------------------------
SCurve::SCurve(const float sample_time) {
    SAMPLE_TIME_ = sample_time;

    start_time_ = 0;

    // Kinematics limits
    X_MAX_ = {0, 0, 0, 0, 0};
    Y_MAX_ = {0, 0, 0, 0, 0};
    Z_MAX_ = {0, 0, 0, 0, 0};
    Kine_MIN_ = {0, 0, 0, 0, 0};

    prev_x_ = {0, 0, 0, 0, 0};
    prev_y_ = {0, 0, 0, 0, 0};
    prev_z_ = {0, 0, 0, 0, 0};

    x_ = {0, 0, 0, 0, 0};
    y_ = {0, 0, 0, 0, 0};
    z_ = {0, 0, 0, 0, 0};
    
    // Time intervals
    T_x_ = {0, 0, 0, 0, 0};
    T_y_ = {0, 0, 0, 0, 0};
    T_z_ = {0, 0, 0, 0, 0};

    // Kinematic profiles (Jerk, Acceleration, Velocity, Position)
    T_x_ = {0, 0, 0, 0};
    T_y_ = {0, 0, 0, 0};
    T_z_ = {0, 0, 0, 0};
    
    // Starting point
    start_ = {0, 0, 0};

    // Goal point
    goal_ = {0, 0, 0};

}

// DESCTRUCTOR ------------------------------------------
SCurve::~SCurve(){}

// METHODS -------------------------------------------------------------
void SCurve::setStartAndGoal(const std::array<float,3>& start, const std::array<float,3>& goal){
    start_ = start;
    goal_  = goal;
}


void SCurve::setKinematicConstraints(const std::array<float,5>& x_max, const std::array<float,5>& y_max, const std::array<float,5>& z_max){
    // SURGE kinematics limits
    X_MAX_ = x_max;

    // Sway kinematics limits
    Y_MAX_ = y_max;

    // Heave kinematics limits
    Z_MAX_ = z_max;

    Kine_MIN_ = Y_MAX_; // Y is the DOF with the minimum kinematics
}

void SCurve::calculateTimeIntervals(const KinematicVar_& var){

    float distance;
    // At this point, kinematic values of all DOFs should be the same, so
    // using the limits of the first DOF is valid, except for the distance
    switch(var){
        case SURGE:
            distance = X_MAX_[0];
            break;
        case SWAY:
            distance = Y_MAX_[0];
            break;
        case HEAVE:
            distance = Z_MAX_[0];
            break;
        default:
            break;
    }
    float V_max = Kine_MIN_[1];
    float A_max = Kine_MIN_[2];
    float J_max = Kine_MIN_[3];
    float S_max = Kine_MIN_[4];

    /* Calculation of time parameters */
    // STEP 1: Determination of the varying jerk phase duration Ts
    float j_max = 0;

    double Ts = 0.0;
    double Tj = 0.0;
    double Ta = 0.0;
    double Tv = 0.0;

    double Ta_d = 0;
    double Tv_d = 0;
    double Ta_v = 0;
    double Tj_d = 0;
    double Tj_v = 0;
    double Tj_a = 0;

    double Ts_d = std::pow(std::sqrt(3)*std::fabs(distance)/(8*S_max), 0.25);
    double Ts_v = std::cbrt(std::sqrt(3)*V_max/(2*S_max));
    double Ts_a = std::sqrt(std::sqrt(3)*A_max/S_max);
    double Ts_j = std::sqrt(3)*J_max/S_max;
    Ts = std::min({Ts_d, Ts_v, Ts_a, Ts_j});

    // ROS_INFO_STREAM("Ts_v = " << Ts_v << ", Ts_a = " << Ts_a << ", Ts_s = " << Ts_j);
    // ROS_INFO_STREAM("Ts = " << Ts_);

    // Case 1
    if(Ts == Ts_d){
        Tj = 0;
        Ta = 0;
        Tv = 0;
        j_max = S_max*Ts_d/std::sqrt(3);
    }
    // Case 2
    else if(Ts == Ts_v){
        Tj = 0;
        Ta = 0;
        j_max = S_max*Ts_v/std::sqrt(3);
        // Step 4
        Tv = (std::fabs(distance)/V_max) - (4*Ts + 2*Tj + Ta);
    }
    // Case 3
    else if(Ts == Ts_a){
        Tj = 0;
        j_max = S_max*Ts_a/std::sqrt(3);
        // Step 3
        Ta_d = (-(6*Ts + 3*Tj) + std::sqrt(std::pow(2*Ts + Tj,2) + 4*std::fabs(distance)/A_max))/2;
        Ta_v = V_max/A_max - 2*Ts - Tj;
        Ta = std::min({Ta_d, Ta_v});

        // Case 1
        if(Ta == Ta_d){
            Tv = 0;
        }
        // Case 2
        else{
        // Step 4
            Tv = (std::fabs(distance)/V_max) - (4*Ts + 2*Tj + Ta);
        }
    }
    // Case 4
    else {
        float term1 = std::cbrt(std::pow(Ts,3)/27 + std::fabs(distance)/(4*J_max)
                     + std::sqrt(std::fabs(distance)*std::pow(Ts,3)/(54*J_max)
                     + std::pow(distance,2)/(16*std::pow(J_max,2))));
        float term2 = std::cbrt(std::pow(Ts,3)/27 + std::fabs(distance)/(4*J_max)
                     - std::sqrt(std::fabs(distance)*std::pow(Ts,3)/(54*J_max)
                     + std::pow(distance,2)/(16*std::pow(J_max,2))));
        Tj_d = term1 + term2 - 5*Ts/3;
        j_max = J_max;
        Tj_v = -3*Ts/2 + std::sqrt(std::pow(Ts,2)/4 + V_max/J_max);
        Tj_a = A_max/J_max - Ts;
        Tj = std::min({Tj_d, Tj_v, Tj_a});

        // Case 1
        if(Tj == Tj_d){
            Ta = 0;
            Tv = 0;
        }
        // Case 2
        else if(Tj == Tj_v){
            // Step 4
            Ta = 0;
            Tv = (std::fabs(distance)/V_max) - (4*Ts + 2*Tj + Ta);
        }
        // Case 3
        else {
            // Step 3
            Ta_d = (-(6*Ts + 3*Tj) + std::sqrt(std::pow(2*Ts + Tj,2) + 4*std::fabs(distance)/A_max))/2;
            Ta_v = V_max/A_max - 2*Ts - Tj;
            Ta = std::min({Ta_d, Ta_v});
            // Case 1
            if(Ta == Ta_d){
                Tv = 0;
            // Case 2
            }
            else{
                // Step 4
                Tv = (std::fabs(distance)/V_max) - (4*Ts + 2*Tj + Ta);
            }
        }
    }

    double T_exe = 8*Ts + 4*Tj + 2*Ta + Tv;

    // Calculation of kinematic values
    // j_max = S_max*Ts/std::sqrt(3);
    float a_max = S_max*Ts*(Ts + Tj)/std::sqrt(3);
    float v_max = S_max*Ts*(Ts + Tj)*(2*Ts + Tj + Ta)/std::sqrt(3);
    // %d_max = sign(distance)S_max*Ts(Ts + Tj)(2*Ts + Tj + Ta)(4*Ts + 2*Tj + Ta + Tv)/std::sqrt(3);

    switch(var){
        case SURGE:
            X_MAX_ = {distance, v_max, a_max, j_max, S_max};
            T_x_ = {T_exe, Tv, Ta, Tj, Ts};
            break;
        case SWAY:
            Y_MAX_ = {distance, v_max, a_max, j_max, S_max};
            T_y_ = {T_exe, Tv, Ta, Tj, Ts};
            break;
        case HEAVE:
            Z_MAX_ = {distance, v_max, a_max, j_max, S_max};
            T_z_ = {T_exe, Tv, Ta, Tj, Ts};
            break;
        default:
            break;
    }

    // ROS_INFO_STREAM("For kinematic " << var);
    // ROS_INFO_STREAM("T exe = " << T_exe << ", T_v = " << Tv << ", T_a = " << Ta << ", T_j = " << Tj << ", Ts = " << Ts);
}

float SCurve::calculateJerk(double current_time, const KinematicVar_& var){

    float J_max;
    float distance;

    double Ts;
    double Tj;
    double Ta;
    double Tv;
    double t0 = start_time_;
    double t = current_time;

    switch(var){
        case SURGE:
            J_max = X_MAX_[3];
            Ts = T_x_[4];
            Tj = T_x_[3];
            Ta = T_x_[2];
            Tv = T_x_[1];
            break;
        case SWAY:
            J_max = Y_MAX_[3];
            Ts = T_y_[4];
            Tj = T_y_[3];
            Ta = T_y_[2];
            Tv = T_y_[1];
            break;
        case HEAVE:
            J_max = Z_MAX_[3];
            Ts = T_z_[4];
            Tj = T_z_[3];
            Ta = T_z_[2];
            Tv = T_z_[1];
            break;
        default:
            break;
    }

    float a = std::sqrt(3)/2;

    double t1 = t0 + Ts;
    double t2 = t1 + Tj;
    double t3 = t2 + Ts;
    double t4 = t3 + Ta;
    double t5 = t4 + Ts;
    double t6 = t5 + Tj;
    double t7 = t6 + Ts;
    double t8 = t7 + Tv;
    double t9 = t8 + Ts;
    double t10 = t9 + Tj;
    double t11 = t10 + Ts;
    double t12 = t11 + Ta;
    double t13 = t12 + Ts;
    double t14 = t13 + Tj;
    double t15 = t14 + Ts;
    float j = 0;
    double tau_i = 0;

    // ROS_INFO_STREAM("Current time = " << t - start_time_);
    // ROS_INFO_STREAM("t0 = " << t0);
    // ROS_INFO_STREAM("t1 = " << t1 - start_time_);
    // ROS_INFO_STREAM("t2 = " << t2 - start_time_);
    // ROS_INFO_STREAM("t3 = " << t3 - start_time_);
    // ROS_INFO_STREAM("t4 = " << t4 - start_time_);
    // ROS_INFO_STREAM("t5 = " << t5 - start_time_);
    // ROS_INFO_STREAM("t6 = " << t6 - start_time_);
    // ROS_INFO_STREAM("t7 = " << t7 - start_time_);
    // ROS_INFO_STREAM("t8 = " << t8 - start_time_);
    // ROS_INFO_STREAM("t9 = " << t9 - start_time_);
    // ROS_INFO_STREAM("t10 = " << t10 - start_time_);
    // ROS_INFO_STREAM("t11 = " << t11 - start_time_);
    // ROS_INFO_STREAM("t12 = " << t12 - start_time_);
    // ROS_INFO_STREAM("t13 = " << t13 - start_time_);
    // ROS_INFO_STREAM("t14 = " << t14 - start_time_);
    // ROS_INFO_STREAM("t15 = " << t15 - start_time_);

    if (t >= t0 && t < t1){
        tau_i = (t-t0)/(t1-t0);
        j = J_max/(1+std::exp(-a*(1/(1-tau_i) - 1/tau_i)));
        // ROS_INFO_STREAM("1");
    }
    else if (t >= t1 && t < t2){
        j = J_max;
        // ROS_INFO_STREAM("2");
    }
    else if (t >= t2 && t < t3){
        tau_i = (t-t2)/(t3-t2);
        j = J_max/(1+std::exp(a*(1/(1-tau_i) - 1/tau_i)));
        // ROS_INFO_STREAM("3");
    }
    else if (t >= t3 && t < t4){
        j = 0;
        // ROS_INFO_STREAM("4");
    }
    else if (t >= t4 && t < t5){
        tau_i = (t-t4)/(t5-t4);
        j = -J_max/(1+std::exp(-a*(1/(1-tau_i) - 1/tau_i)));
        // ROS_INFO_STREAM("5");
    }
    else if (t >= t5 && t < t6){
        j = -J_max;
        // ROS_INFO_STREAM("6");
    }
    else if (t >= t6 && t < t7){
        tau_i = (t-t6)/(t7-t6);
        j = -J_max/(1+std::exp(a*(1/(1-tau_i) - 1/tau_i)));
        // ROS_INFO_STREAM("7");
    }
    else if(t >= t7 && t < t8){
        j = 0;
        // ROS_INFO_STREAM("8");
    }
    else if(t >= t8 && t < t9){
        tau_i = (t-t8)/(t9-t8);
        j = -J_max/(1+std::exp(-a*(1/(1-tau_i) - 1/tau_i)));
        // ROS_INFO_STREAM("9");
    }
    else if(t >= t9 && t < t10){
        j = -J_max;
        // ROS_INFO_STREAM("10");
    }
    else if(t >= t10 && t < t11){
        tau_i = (t-t10)/(t11-t10);
        j = -J_max/(1+std::exp(a*(1/(1-tau_i) - 1/tau_i)));
        // ROS_INFO_STREAM("11");
    }
    else if(t >= t11 && t < t12){
        j = 0;
        // ROS_INFO_STREAM("12");
    }
    else if(t >= t12 && t < t13){
        tau_i = (t-t12)/(t13-t12);
        j = J_max/(1+std::exp(-a*(1/(1-tau_i) - 1/tau_i)));
        // ROS_INFO_STREAM("13");
    }
    else if(t >= t13 && t < t14){
        j = J_max;
        // ROS_INFO_STREAM("14");
    }
    else if(t >= t14 && t < t15){
        tau_i = (t-t14)/(t15-t14);
        j = J_max/(1+std::exp(a*(1/(1-tau_i) - 1/tau_i)));
        // ROS_INFO_STREAM("15");
    }

    switch(var){
        case SURGE:
            distance = X_MAX_[0];
            break;
        case SWAY:
            distance = Y_MAX_[0];
            break;
        case HEAVE:
            distance = Z_MAX_[0];
            break;
        default:
            break;
    }
    j = distance/std::abs(distance)*j;

    // ROS_INFO_STREAM("J = " << j);
    // ROS_INFO_STREAM("Distance = " << distance);
    return j;
}

void SCurve::calculateTrajectory(const double current_time){
    
    X_MAX_[0] = goal_[0] - start_[0];
    Y_MAX_[0] = goal_[1] - start_[1];
    Z_MAX_[0] = goal_[2] - start_[2];

    // Calculate times for all DOF
    calculateTimeIntervals(SURGE);
    calculateTimeIntervals(SWAY);
    calculateTimeIntervals(HEAVE);

    prev_x_ = x_;
    prev_y_ = y_;
    prev_z_ = z_;

    // Calculate trajectories (it is the same jerk for every DOF)
    x_[3] += calculateJerk(current_time, SURGE);
    y_[3] += calculateJerk(current_time, SWAY);
    z_[3] += calculateJerk(current_time, HEAVE);

    // ROS_INFO_STREAM("jx = " << x_[3] << ", jy = " << y_[3] << ", jz = " << z_[3]);

    // Calculate acceleration
    x_[2] += (prev_x_[3] + x_[3])/2*SAMPLE_TIME_;
    y_[2] += (prev_y_[3] + y_[3])/2*SAMPLE_TIME_;
    z_[2] += (prev_z_[3] + z_[3])/2*SAMPLE_TIME_;

    // ROS_INFO_STREAM("ax = " << x_[2] << ", ay = " << y_[2] << ", az = " << z_[2]);

    // Calculate velocity
    x_[1] += (prev_x_[2] + x_[2])/2*SAMPLE_TIME_;
    y_[1] += (prev_y_[2] + y_[2])/2*SAMPLE_TIME_;
    z_[1] += (prev_z_[2] + z_[2])/2*SAMPLE_TIME_;

    // Calculate position
    x_[0] += (prev_x_[1] + x_[1])/2*SAMPLE_TIME_;
    y_[0] += (prev_y_[1] + y_[1])/2*SAMPLE_TIME_;
    z_[0] += (prev_z_[1] + z_[1])/2*SAMPLE_TIME_;

    // Tsync = std::max([Tx(1), Ty(1), Tz(1)]);
}

vanttec_msgs::Trajectory SCurve::getTrajectory(){
    vanttec_msgs::Trajectory trajectory;
    
    trajectory.jerk.linear.x = x_[3];
    trajectory.jerk.linear.y = y_[3];
    trajectory.jerk.linear.z = z_[3];

    trajectory.accel.linear.x = x_[2];
    trajectory.accel.linear.y = y_[2];
    trajectory.accel.linear.z = z_[2];

    trajectory.vel.linear.x = x_[1];
    trajectory.vel.linear.y = y_[1];
    trajectory.vel.linear.z = z_[1];

    trajectory.eta_pose.x = x_[0];
    trajectory.eta_pose.y = y_[0];
    trajectory.eta_pose.z = z_[0];

    return trajectory;
}

void SCurve::setStartTime(const double start_time){
    start_time_ = start_time;
}