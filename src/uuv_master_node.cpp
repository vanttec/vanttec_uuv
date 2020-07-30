#include "master_node.hpp"

#include <ros/ros.h>
#include <stdio.h>

const float SAMPLE_TIME_S       = 0.01;
const float DEFAULT_SPEED_MPS   = 0.2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_master_node");
    ros::NodeHandle nh;
    
    ros::Rate           cycle_rate(int(1 / SAMPLE_TIME_S));
    UUVMasterNode       uuv_master(DEFAULT_SPEED_MPS);
    
    std::cout << uuv_master.default_speed << std::endl;
    
    ros::Publisher  uuv_vel      = nh.advertise<geometry_msgs::Twist>("/uuv_control/uuv_control_node/setpoint", 1000);
    ros::Publisher  uuv_status   = nh.advertise<vanttec_uuv::MasterStatus>("/uuv_master/uuv_master_node/status", 1000);
    ros::Publisher  uuv_estop    = nh.advertise<std_msgs::Empty>("/uuv_master/uuv_master_node/e_stop", 1000);

    ros::Subscriber kb_up_key    = nh.subscribe("/vehicle_user_control/vehicle_user_control/kb_keyup",
                                                1000,
                                                &UUVMasterNode::keyboardUpCallback,
                                                &uuv_master);

    ros::Subscriber kb_down_key  = nh.subscribe("/vehicle_user_control/vehicle_user_control/kb_keydown",
                                                10,
                                                &UUVMasterNode::keyboardDownCallback,
                                                &uuv_master);

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* Publish Data */ 
        if (uuv_master.publish_flag == 1)
        {
            uuv_vel.publish(uuv_master.velocities);
            uuv_master.publish_flag = 0;
        }
        uuv_status.publish(uuv_master.status);

        //std::cout << uuv_master.velocities << std::endl;
        
        if (uuv_master.e_stop_flag == 1)
        {
            uuv_master.e_stop_flag = 0;
            uuv_estop.publish(uuv_master.emergency_stop);
        }

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}