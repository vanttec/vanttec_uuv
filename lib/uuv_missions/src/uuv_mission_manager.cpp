/** ----------------------------------------------------------------------------
 * @file: uuv_mission_manager.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: Mission manager class, used to trigger and track mission progress.
 * -----------------------------------------------------------------------------
 * */

#include <uuv_mission_manager.hpp>
#include <iostream>

MissionManager::MissionManager()
{
    this->selected_side                     = LEFT;
    this->current_mode                      = MANUAL;

    this->current_pose.position.x           = 0;
    this->current_pose.position.y           = 0;
    this->current_pose.position.z           = 0;
    this->current_pose.orientation.z        = 0;

    this->mission_status.current_mission    = 0;
    this->mission_status.current_state      = 0;

    this->detected_obstacles.obstacles.clear();

    this->ResetWaypoints();

    // Mission pointer initializations

    this->gate_mission              = nullptr;
    this->buoy_mission              = nullptr;
    this->shootout_mission          = nullptr;
    this->bin_mission               = nullptr;
}

MissionManager::~MissionManager(){}

void MissionManager::OnObstacleReception(const vanttec_uuv::DetectedObstacles& _obstacles)
{
    this->detected_obstacles.obstacles = _obstacles.obstacles;
}

void MissionManager::OnMasterStatusReception(const vanttec_uuv::MasterStatus& _status)
{
    if (this->current_mode != (Mode_E) _status.status)
    {
        this->mission_status.current_mission = 0;
        this->mission_status.current_state = 0;
        this->current_mode = (Mode_E) _status.status;
    }
}

void MissionManager::OnMissionConfigReception(const vanttec_uuv::MissionStatus& _mission)
{
    if (((MissionType_E) this->mission_status.current_mission) == NONE)
    {
        this->mission_status.current_mission = _mission.current_mission;
        this->mission_status.selected_side = _mission.selected_side;
        this->selected_side = (Side_E) _mission.selected_side;

        switch((MissionType_E) this->mission_status.current_mission)
        {
            case GATE:
                this->gate_mission = new GateMission::GateMission;
                break;
            case BUOY:
                this->buoy_mission = new BuoyMission::BuoyMission;
                break;
            case TORPEDOES:
                this->shootout_mission = new ShootOutMission::ShootOutMission;
                break;
            case BIN:
                this->bin_mission = new BinMission::BinMission;
                break;
            case LOS_NAV_TEST:
            case ORBIT_NAV_TEST:
            default:
                break;
        }
    }
}

void MissionManager::OnPoseReception(const geometry_msgs::Pose& _pose)
{
    this->current_pose.position.x       = _pose.position.x;
    this->current_pose.position.y       = _pose.position.y;
    this->current_pose.position.z       = _pose.position.z;
    this->current_pose.orientation.z    = _pose.orientation.z;

}

void MissionManager::OnEStopReception(const std_msgs::Empty& _empty)
{
    this->mission_status.current_mission    = 0;
    this->mission_status.current_state      = 0;
}

void MissionManager::UpdateStateMachines()
{
    if (this->current_mode == MANUAL)
    {
        this->mission_status.current_mission = 0;
        this->mission_status.current_state = 0;
    }
    else
    {
        switch((MissionType_E) this->mission_status.current_mission)
        {
            case NONE:
                this->ResetWaypoints();
                this->mission_status.current_state = 0;
                break;
            case GATE:
                if (this->gate_mission != nullptr)
                {
                    this->gate_mission->UpdateStateMachine(&this->selected_side, 
                                                           &this->detected_obstacles, 
                                                           &this->current_pose,
                                                           &this->desired_waypoints);

                    this->mission_status.current_state = (int) this->gate_mission->state_machine;

                    if (this->gate_mission->state_machine == GateMission::DONE)
                    {
                        delete this->gate_mission;
                        this->gate_mission = nullptr;
                        this->mission_status.current_mission = NONE;
                        this->ResetWaypoints();
                    }
                }
                break;
            case BUOY:
                if (this->buoy_mission != nullptr)
                {
                    this->buoy_mission->UpdateStateMachine(&this->selected_side, 
                                                           &this->detected_obstacles, 
                                                           &this->current_pose,
                                                           &this->desired_waypoints);

                    this->mission_status.current_state = (int) this->buoy_mission->state_machine;

                    if (this->buoy_mission->state_machine == BuoyMission::DONE)
                    {
                        delete this->buoy_mission;
                        this->buoy_mission = nullptr;
                        this->mission_status.current_mission = NONE;
                        this->ResetWaypoints();
                    }
                }
                break;
            case TORPEDOES:
                if (this->shootout_mission != nullptr)
                {
                    this->shootout_mission->UpdateStateMachine(&this->selected_side, 
                                                                &this->detected_obstacles, 
                                                                &this->current_pose,
                                                                &this->desired_waypoints);

                    this->mission_status.current_state = (int) this->shootout_mission->state_machine;

                    if (this->shootout_mission->state_machine == BuoyMission::DONE)
                    {
                        delete this->shootout_mission;
                        this->shootout_mission = nullptr;
                        this->mission_status.current_mission = NONE;
                        this->ResetWaypoints();
                    }
                }
            case BIN:
                if (this->bin_mission != nullptr)
                {
                    this->bin_mission->UpdateStateMachine(&this->selected_side, 
                                                                &this->detected_obstacles, 
                                                                &this->current_pose,
                                                                &this->desired_waypoints);

                    this->mission_status.current_state = (int) this->bin_mission->state_machine;

                    if (this->bin_mission->state_machine == BinMission::DONE)
                    {
                        delete this->bin_mission;
                        this->bin_mission = nullptr;
                        this->mission_status.current_mission = NONE;
                        this->ResetWaypoints();
                    }
                }
            case LOS_NAV_TEST:
            case ORBIT_NAV_TEST:
            default:
                break;
        }
    }
    
}

void MissionManager::ResetWaypoints()
{
    this->desired_waypoints.guidance_law = 0;
    this->desired_waypoints.waypoint_list_length = 2;
    this->desired_waypoints.waypoint_list_x = {0,0};
    this->desired_waypoints.waypoint_list_y = {0,0};
    this->desired_waypoints.waypoint_list_z = {0,0};
    this->desired_waypoints.depth_setpoint = 0;
    this->desired_waypoints.heading_setpoint = 0;
}