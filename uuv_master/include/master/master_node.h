#ifndef MASTER_NODE_H_
#define MASTER_NODE_H_
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <uuv_master/UUVMasterStatus.h>
#include "keyboard.hpp"


class MasterNode
{
    public:
    MasterNode(ros::NodeHandle n, ros::NodeHandle n_private);

    private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber key_up;                // Topic for keyboard input-up
    ros::Subscriber key_down;              // Topic for keyboard input-down
    ros::Publisher velocity_pub_;          // Topic to monitor velocity
    ros::Publisher emergency_stop_pub_;    // Topic to stop machine
    ros::Publisher operation_mode_pub_;    // Topic of operation
    geometry_msgs::Twist velocity;         // Message sent for desired velocity   
    std_msgs::Empty emergency_stop;        // Message sent for emergency stop
    std_msgs::UInt8 operation_mode;        // Message sent for operation mode
    vehicle_user_control::KeyboardKey upkey; // Message used for keyup
    vehicle_user_control::KeyboardKey downkey; // Message used for keydown
    void keyboardUPCallBack(const vehicle_user_control::KeyboardKey::ConstPtr& msg);
    void keyboardDOWNCallBack(const vehicle_user_control::KeyboardKey::ConstPtr& msg));
    void sendCommands(void);
    std_msgs::Empty empty_msg;
    uuv_master::UUVMasterStatus key_message;
    std_msgs::Twist velocity;
   
}

#endif // MASTER_NODE_H_