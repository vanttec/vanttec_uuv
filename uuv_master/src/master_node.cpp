#include <master/master_node.h>
#include <ros/package.h> 
#include <ros/ros.h>
#include <iostream>

//Constructor
MasterNode::MasterNode(ros::NodeHandle n, ros::NodeHandle n_private)
{
    nh_ = n;
    nh_private_ = n_private;
    // Subscribe to receive keyboard input
    key_down_ = nh_.subscribe("vehicle_user_control/kb_keydown", 10, &MasterNode::keyboardUPCallBack, this);
    key_up_ = nh_.subscribe("vehicle_user_control/kb_keyup", 10, &MasterNode::keyboardDOWNCallBack, this);
    // Advertise the topics we publish
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/uuv_master/velocity", 10);
    emergency_stop_pub_ = nh_.advertise<std_msgs::Empty>("/uuv_master/emergency_stop_pub_", 10);
    operation_mode_pub_ = nh_.advertise<vehicle_user_control::KeyboardKey>("/uuv_master/operation_mode_pub_", 10);

}

void MasterNode::keyboardUPCallBack(const vehicle_user_control::KeyboardKey::ConstPtr& msg)
{
    upkey = msg->code;
}
void MasterNode::keyboardDOWNCallBack(const vehicle_user_control::KeyboardKey::ConstPtr& msg)
{
    downkey = msg->code;
}
void MasterNode::sendCommands(void)
{
    if(downkey == vehicle_user_control::KeyboardKey::KEY_SPACE){
        velocity.angular.x = 0.0;//Roll
        velocity.angular.y = 0.0;//Pitch
        velocity.angular.z = 0.0;//Yaw
        velocity.linear.x = 0.0;//x linear movement
        velocity.linear.y = 0.0;//y linear movement
        velocity.linear.z = 0.0;//z linear movement
    }
    else {
    if(downkey == vehicle_user_control::KeyboardKey::KEY_SPACE){
        //Emergency stop
        key_message.DESIREDROUTINE = 0;
        emergency_stop_pub_.publish(empty_msg);
        operation_mode_pub_.publish(key_message);
    }
    else{

            if(downkey == vehicle_user_control::KeyboardKey::KEY_E){
                //Toogle Motors
                if((downkey == vehicle_user_control::KeyboardKey::KEY_1) || (downkey == vehicle_user_control::KeyboardKey::KEY_2)||(downkey == vehicle_user_control::KeyboardKey::KEY_3)|| (downkey == vehicle_user_control::KeyboardKey::KEY_4)||(downkey == vehicle_user_control::KeyboardKey::KEY_5)){
                    //Autonomous Mode
                    key_message.STATUS = 1;

                    if(downkey == vehicle_user_control::KeyboardKey::KEY_1){
                    //Routine 1
                    key_message.DESIREDROUTINE = 1;
                    operation_mode_pub_.publish(key_message);
                    }
                    else if(downkey == vehicle_user_control::KeyboardKey::KEY_2){
                    //Routine 2
                    key_message.DESIREDROUTINE = 2;
                    operation_mode_pub_.publish(key_message);
                    }
                    else if(downkey == vehicle_user_control::KeyboardKey::KEY_3){
                    //Routine 3
                    key_message.DESIREDROUTINE = 3;
                    operation_mode_pub_.publish(key_message);
                    }
                    else if(downkey == vehicle_user_control::KeyboardKey::KEY_4){
                    //Routine 4
                    key_message.DESIREDROUTINE = 4;
                    operation_mode_pub_.publish(key_message);
                    }          
                    else{
                    //Routine 5
                    key_message.DESIREDROUTINE = 5;
                    operation_mode_pub_.publish(key_message);
                    }      
                }
                else{
                    //Manual Mode
                    key_message.STATUS = 0;
                    if(downkey == vehicle_user_control::KeyboardKey::KEY_W){
                    //Surge+
                    velocity.linear.x = const_velocity_;

                    }
                    if(downkey == vehicle_user_control::KeyboardKey::KEY_S){
                    //Surge-
                    velocity.linear.x = -const_velocity_;

                    }
                    if(downkey == vehicle_user_control::KeyboardKey::KEY_A){
                    //Sway-
                    velocity.linear.y = const_velocity_;
                    }
                    if(downkey == vehicle_user_control::KeyboardKey::KEY_D){
                    //Sway+
                    velocity.linear.y = -const_velocity_;
                    }
                    if(downkey == vehicle_user_control::KeyboardKey::KEY_LSHIFT){
                    //Heave-
                    velocity.linear.z = -const_velocity_; 
                    }   
                    if(downkey == vehicle_user_control::KeyboardKey::KEY_LCTRL){
                    //Heave+
                    velocity.linear.z = const_velocity_;    
                    }  
                    if(downkey == vehicle_user_control::KeyboardKey::KEY_LEFT){
                    //Yaw-
                    velocity.angular.z = -const_velocity_;
                    }   
                    if(downkey == vehicle_user_control::KeyboardKey::KEY_RIGHT){
                    //Yaw+
                    velocity.angular.z = const_velocity_;
                    }   
                    
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MasterNode");
    ros::NodeHandle nh;
    ros::NOdeHandel nh_private("~");
    
    MasterNode master_node(nh, nh_private);
    std::string node_name = ros::this_node::getName();
    ROS_INFO("%s started", node_name.c_str());

    ros::Rate r(10); //10Hz

    while(ros::ok())
    {
        master_node.sendComands();
        ros::spinOnce();
        r.sleep();
    }

    return 0;

}
