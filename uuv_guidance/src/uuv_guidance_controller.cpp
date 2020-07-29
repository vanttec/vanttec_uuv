#include <uuv_guidance_controller.hpp>

GuidanceController::GuidanceController()
{
    /* Desired speed output initalization */
    this->desired_setpoints.linear.x = 0;
    this->desired_setpoints.linear.y = 0;
    this->desired_setpoints.linear.z = 0;
    this->desired_setpoints.angular.z = 0;

    /* State Machines Initialization */
    this->current_guidance_law = NONE;
    this->los_state_machine.state_machine = LOS_LAW_STANDBY;

    /* LOS Parameter Init */
    this->los_state_machine.current_waypoint = 0;
    this->los_depth_error_threshold = 0.01;
    this->los_lookahead_distance = 0.65; // Lookahead distance corresponds to 2 times the length of the UUV
    this->los_min_speed = 0;
    this->los_max_speed = 0.9;
    this->los_position_error_threshold = 0.25;
    this->euclidean_distance = 0;
    this->los_speed_gain = 50;
}

GuidanceController::~GuidanceController(){}
        
void GuidanceController::OnCurrentPositionReception(const geometry_msgs::Pose& _pose)
{
    /* Store the current position in NED coordinates */
    this->current_positions_ned.position.x      = _pose.position.x;
    this->current_positions_ned.position.y      = _pose.position.y;
    this->current_positions_ned.position.z      = _pose.position.z;
    this->current_positions_ned.orientation.z   = _pose.orientation.z;
}

void GuidanceController::OnWaypointReception(const uuv_guidance::GuidanceWaypoints& _waypoints)
{
    /* Waypoints update (and therefore, guidance law triggering) can only be done when the guidance
    node is not executing any other type of action/law; only acceptable input is an emergency stop */
    
    switch(this->current_guidance_law)
    {
        case NONE:
            
            /* Trigger the appropriate guidance law state machine */
            switch((GuidanceLaws_E)_waypoints.guidance_law)
            {
                case LOS_GUIDANCE_LAW:
                    this->los_state_machine.state_machine = LOS_LAW_DEPTH_NAV;
                    break;
                case ORBIT_GUIDANCE_LAW:
                case NONE:
                default:
                    break;
            }

            /* Update the current guidance law selection and the internal waypoint list */
            this->current_guidance_law = (GuidanceLaws_E) _waypoints.guidance_law;
            this->current_waypoint_list = _waypoints;
            break;

        case LOS_GUIDANCE_LAW:
        case ORBIT_GUIDANCE_LAW:
        default:
            break;
    }
}

void GuidanceController::OnEmergencyStop(const std_msgs::Empty& _msg)
{
    /* Stop the vehicle from moving. Depth and heading keep the previous setpoint. */
    this->desired_setpoints.linear.x = 0;
    this->desired_setpoints.linear.y = 0;

    /* Reset the guidance law and the state machines */
    this->current_guidance_law = NONE;
    this->los_state_machine.state_machine = LOS_LAW_STANDBY;
}


void GuidanceController::UpdateStateMachines()
{
    /* Enter a specific state machine according to the selected guidance law. */
    switch(this->current_guidance_law)
    {
        /* Line-Of-Sight Guidance Law 
           Strategy:
                - Navigate to the specified waypoint depth so that 2D LOS can be used.
                - Compute the desired speed and heading according to the LOS algorithm.
                - When we are in the vicinity of the target waypoint (i.e. euclidean distance < threshold), stop.
                - If there are more waypoints, continue with the next. If not, return to standby.
        */

        case LOS_GUIDANCE_LAW:

            switch(this->los_state_machine.state_machine)
            {
                case LOS_LAW_STANDBY:
                {
                    this->desired_setpoints.linear.x = 0;
                    this->desired_setpoints.linear.y = 0;
                    this->los_state_machine.current_waypoint = 0;
                    break;
                }
                case LOS_LAW_DEPTH_NAV:
                {
                    this->desired_setpoints.linear.x = 0;
                    this->desired_setpoints.linear.y = 0;
                    this->desired_setpoints.angular.z = 0;
                    this->desired_setpoints.linear.z = this->current_waypoint_list.waypoint_list_z[this->los_state_machine.current_waypoint + 1];
                    float los_depth_error = abs((float) this->current_positions_ned.position.z - (float) this->desired_setpoints.linear.z);
                    if (los_depth_error <= this->los_depth_error_threshold)
                    {
                        this->los_state_machine.state_machine = LOS_LAW_WAYPOINT_NAV;
                    }
                    break;
                }
                case LOS_LAW_WAYPOINT_NAV:
                {
                    /* Create references for readability */
                    float x_k  = this->current_waypoint_list.waypoint_list_x[this->los_state_machine.current_waypoint];
                    float x_k1 = this->current_waypoint_list.waypoint_list_x[this->los_state_machine.current_waypoint + 1];

                    float y_k  = this->current_waypoint_list.waypoint_list_y[this->los_state_machine.current_waypoint];
                    float y_k1 = this->current_waypoint_list.waypoint_list_y[this->los_state_machine.current_waypoint + 1];

                    float x_uuv = this->current_positions_ned.position.x;
                    float y_uuv = this->current_positions_ned.position.y;

                    /* Algorithm */
                    float alpha_k = atan2((y_k1 - y_k), (x_k1 - x_k));
                    if (abs(alpha_k) > PI)
                    {
                        alpha_k = (alpha_k/abs(alpha_k)) * (abs(alpha_k) - 2 * PI);
                    }
                    float along_track_distance = (x_uuv - x_k) * cos(alpha_k) + (y_uuv - y_k) * sin(alpha_k);
                    float cross_track_error = - (x_uuv - x_k) * sin(alpha_k) + (y_uuv - y_k) * cos(alpha_k);
                    
                    float total_distance = (x_k1 - x_k) * cos(alpha_k) + (y_k1 - y_k) * sin(alpha_k);
                    /*
                    if (along_track_distance > total_distance)
                    {
                        alpha_k = alpha_k - PI;
                        if (abs(alpha_k) > PI)
                        {
                            alpha_k = (alpha_k/abs(alpha_k)) * (abs(alpha_k) - 2 * PI);
                        }
                        along_track_distance = (x_uuv - x_k) * cos(alpha_k) + (y_uuv - y_k) * sin(alpha_k);
                        cross_track_error = - (x_uuv - x_k) * sin(alpha_k) + (y_uuv - y_k) * cos(alpha_k);
                    }
                    */
                    float desired_heading = alpha_k + atan(-(cross_track_error/this->los_lookahead_distance));

                    if (abs(desired_heading) > PI)
                    {
                        desired_heading = (desired_heading/abs(desired_heading)) * (abs(desired_heading) - 2 * PI);
                    }

                    float desired_velocity = (this->los_max_speed - this->los_min_speed) * 
                                             (1 - (abs(along_track_distance) / sqrt(pow(along_track_distance, 2) + this->los_speed_gain)));

                    if (abs(this->current_positions_ned.orientation.z - desired_heading) > 0.75)
                    {
                        float desired_velocity = 0.075;
                    }
                    
                    this->desired_setpoints.linear.x = desired_velocity;
                    this->desired_setpoints.linear.y = 0;
                    this->desired_setpoints.angular.z = desired_heading;

                    this->euclidean_distance = sqrt(pow((x_k1 - x_uuv), 2) + pow((y_k1 - y_uuv), 2));

                    if (this->euclidean_distance <= this->los_position_error_threshold)
                    {
                        this->desired_setpoints.linear.x = 0;
                        this->desired_setpoints.linear.y = 0;
                        this->desired_setpoints.angular.z = 0;
                        this->euclidean_distance = 0;
                        
                        if ((this->los_state_machine.current_waypoint + LOS_WAYPOINT_OFFSET) < this->current_waypoint_list.waypoint_list_length)
                        {
                            this->los_state_machine.current_waypoint += 1;
                            this->los_state_machine.state_machine = LOS_LAW_DEPTH_NAV;
                        }
                        else
                        {
                            this->los_state_machine.state_machine = LOS_LAW_STANDBY;
                            this->los_state_machine.current_waypoint = 0;
                            this->current_guidance_law = NONE;
                        }
                    }
                    break;
                }
            }
            break;

        /* Orbit Guidance Law 
           Strategy:
                - 
        */
        case ORBIT_GUIDANCE_LAW:
            break;

        /* If no guidance law is being executed, keep desired speeds at 0 and maintain current depth and heading. */
        case NONE:
        default:
            break;
    }
}
