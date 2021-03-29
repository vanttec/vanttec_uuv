/** ----------------------------------------------------------------------------
 * @file: uuv_mission_manager_node.cpp
 * @date: July 30, 2020
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * 
 * @brief: ROS mission manager node for the UUV. Uses uuv_missions library.
 * -----------------------------------------------------------------------------
 **/

#include "uuv_mission_manager.hpp"

#include <ros/ros.h>
#include <stdio.h>

static const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_mission_manager");
    ros::NodeHandle nh;
        
    ros::Rate               cycle_rate(int(1 / SAMPLE_TIME_S));
    MissionManager          manager;

    ros::Publisher  waypoints   = nh.advertise<vanttec_uuv::GuidanceWaypoints>("/uuv_guidance/guidance_controller/waypoints", 1000);
    ros::Publisher  status      = nh.advertise<vanttec_uuv::MissionStatus>("/uuv_missions/manager/status", 1000);


    ros::Subscriber uuv_pose = nh.subscribe("/uuv_simulation/dynamic_model/pose", 
                                            10, 
                                            &MissionManager::OnPoseReception, 
                                            &manager);
    
    ros::Subscriber obstacles = nh.subscribe("/object_simulator/detected_objects", 
                                            10, 
                                            &MissionManager::OnObstacleReception, 
                                            &manager);
    
    ros::Subscriber uuv_status = nh.subscribe("/uuv_master/uuv_master_node/status", 
                                            10, 
                                            &MissionManager::OnMasterStatusReception, 
                                            &manager);
    
    ros::Subscriber mission_config = nh.subscribe("/uuv_master/uuv_master_node/mission_config", 
                                                  10, 
                                                  &MissionManager::OnMissionConfigReception, 
                                                  &manager);
    
    ros::Subscriber e_stop = nh.subscribe("/uuv_master/uuv_master_node/e_stop", 
                                          10, 
                                          &MissionManager::OnEStopReception, 
                                          &manager);
    
    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* Calculate Model States */
        manager.UpdateStateMachines();

        /* Publish Odometry */
        if (manager.gate_mission != nullptr)
        {
            if (manager.gate_mission->state_machine != GateMission::NAVIGATE && 
                manager.mission_status.current_mission != 0 )
            {   
                waypoints.publish(manager.desired_waypoints);   
            }
        }
        if (manager.buoy_mission != nullptr)
        {
            if (manager.buoy_mission->state_machine != BuoyMission::NAVIGATE && 
                manager.mission_status.current_mission != 0 )
            {   
                waypoints.publish(manager.desired_waypoints);   
            }
        }
        if (manager.shootout_mission != nullptr)
        {
            if (manager.shootout_mission->state_machine != ShootOutMission::NAVIGATE && 
                manager.mission_status.current_mission != 0 )
            {   
                waypoints.publish(manager.desired_waypoints);   
            }
        }
        if (manager.bin_mission != nullptr)
        {
            if (manager.bin_mission->state_machine != BinMission::NAVIGATE && 
                manager.mission_status.current_mission != 0 )
            {   
                waypoints.publish(manager.desired_waypoints);   
            }
        }
        status.publish(manager.mission_status);
        
        /* Sleep for 10ms */
        cycle_rate.sleep();
    }

    return 0; 
}