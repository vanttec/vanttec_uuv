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
    std_msgs::Empty empty_msg;
    vehicle_user_control::KeyboardKey key_message;

    if((upkey == vehicle_user_control::KeyboardKey::KEY_SPACE) || (downkey == vehicle_user_control::KeyboardKey::KEY_SPACE)){
        //Emergency stop
        key_message.DESIREDROUTINE = 0;
        emergency_stop_pub_.publish(empty_msg);
        operation_mode_pub_.publish(key_message);
        velocity_pub_.publish(velocity);
    }
    else{

        if((upkey == vehicle_user_control::KeyboardKey::KEY_E) || (downkey == vehicle_user_control::KeyboardKey::KEY_E)){
            //Toogle Motors
            velocity_pub_.publish(velocity);
            if((upkey == vehicle_user_control::KeyboardKey::KEY_1) || (upkey == vehicle_user_control::KeyboardKey::KEY_2)||(upkey == vehicle_user_control::KeyboardKey::KEY_3) || (upkey == vehicle_user_control::KeyboardKey::KEY_4)||(upkey == vehicle_user_control::KeyboardKey::KEY_5)
               || (downkey == vehicle_user_control::KeyboardKey::KEY_1) || (downkey == vehicle_user_control::KeyboardKey::KEY_2)||(downkey == vehicle_user_control::KeyboardKey::KEY_3)|| (downkey == vehicle_user_control::KeyboardKey::KEY_4)||(downkey == vehicle_user_control::KeyboardKey::KEY_5)){
                //Autonomous Mode
                key_message.STATUS = 1;

                if((upkey == vehicle_user_control::KeyboardKey::KEY_1) || (downkey == vehicle_user_control::KeyboardKey::KEY_1)){
                //Routine 1
                key_message.DESIREDROUTINE = 1;
                operation_mode_pub_.publish(key_message);


                }
                else if((upkey == vehicle_user_control::KeyboardKey::KEY_2) || (downkey == vehicle_user_control::KeyboardKey::KEY_2)){
                //Routine 2

                }
                else if((upkey == vehicle_user_control::KeyboardKey::KEY_3) || (downkey == vehicle_user_control::KeyboardKey::KEY_3)){
                //Routine 3

                }
                else if((upkey == vehicle_user_control::KeyboardKey::KEY_4) || (downkey == vehicle_user_control::KeyboardKey::KEY_4)){
                //Routine 4

                }          
                else{
                //Routine 5

                }      
            }
            else{
                //Manual Mode
                if((upkey == vehicle_user_control::KeyboardKey::KEY_W) || (downkey == vehicle_user_control::KeyboardKey::KEY_W)){
                //Surge+

                }
                if((upkey == vehicle_user_control::KeyboardKey::KEY_A) || (downkey == vehicle_user_control::KeyboardKey::KEY_A)){
                //Sway-

                }
                if((upkey == vehicle_user_control::KeyboardKey::KEY_S) || (downkey == vehicle_user_control::KeyboardKey::KEY_S)){
                //Surge-

                }
                if((upkey == vehicle_user_control::KeyboardKey::KEY_D) || (downkey == vehicle_user_control::KeyboardKey::KEY_D)){
                //Sway+

                }
                if((upkey == vehicle_user_control::KeyboardKey::KEY_LSHIFT) || (downkey == vehicle_user_control::KeyboardKey::KEY_LSHIFT)){
                //Heave-

                }   
                if((upkey == vehicle_user_control::KeyboardKey::KEY_LCTRL) || (downkey == vehicle_user_control::KeyboardKey::KEY_LCTRL)){
                //Heave+

                }  
                if((upkey == vehicle_user_control::KeyboardKey::KEY_LEFT) || (downkey == vehicle_user_control::KeyboardKey::KEY_LEFT)){
                //Yaw-

                }   
                if((upkey == vehicle_user_control::KeyboardKey::KEY_RIGHT) || (downkey == vehicle_user_control::KeyboardKey::KEY_RIGHT)){
                //Yaw+

                }   
                
                velocity.angular.x = ;//Roll
                velocity.angular.y = ;//Pitch
                velocity.angular.z = ;//Yaw
                velocity.linear.x = ;//x linear movement
                velocity.linear.y = ;//y linear movement
                velocity.linear.z = ;//z linear movement
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
