// Original code from: https://github.com/DemianMArin/my_vanttec_uuv.git
// Modified by: Abraham de Jesus Maldonado Mata
// Date: 06/02/2025

#include "can_node_base.h"
#include <iostream>
#include "Vanttec_CANLib/CANMessage.h"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "std_srvs/srv/empty.hpp"
#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Dense>

#include <cstdlib>

using namespace std::chrono_literals;
using std::placeholders::_1;

class CanNodeUUV : public CanNodeBase {
public:

    CanNodeUUV() : CanNodeBase("uuv_can_node"){
        using namespace std::placeholders;

        // Declare parameters
        this->declare_parameter("up_rightMotor", rclcpp::PARAMETER_INTEGER); this->declare_parameter("up_leftMotor", rclcpp::PARAMETER_INTEGER); this->declare_parameter("rightMotor", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("leftMotor", rclcpp::PARAMETER_INTEGER); this->declare_parameter("low_rightMotor", rclcpp::PARAMETER_INTEGER); this->declare_parameter("low_leftMotor", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("scaleControl", rclcpp::PARAMETER_DOUBLE);

        // Get parameters
        up_rightMotor_param = this->get_parameter("up_rightMotor"); up_leftMotor_param = this->get_parameter("up_leftMotor"); rightMotor_param = this->get_parameter("rightMotor");
        leftMotor_param = this->get_parameter("leftMotor"); low_rightMotor_param = this->get_parameter("low_rightMotor"); low_leftMotor_param = this->get_parameter("low_leftMotor");
        scaleControl_param = this->get_parameter("scaleControl");

        // Declaring variables
        up_rightMotor = up_rightMotor_param.as_int(); up_leftMotor = up_leftMotor_param.as_int(); rightMotor = rightMotor_param.as_int();
        low_rightMotor = low_rightMotor_param.as_int(); low_leftMotor = low_leftMotor_param.as_int(); leftMotor = leftMotor_param.as_int();
        scaleControl = scaleControl_param.as_double();

       RCLCPP_INFO(this->get_logger(), "UR: %d, UL: %d, R: %d, L: %d, LR: %d, LL: %d, S: %.2f",up_rightMotor, up_leftMotor, rightMotor, leftMotor, low_rightMotor, low_leftMotor, scaleControl);

        allMotor_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/uuv/allMotor/teleop", 10, [this](const std_msgs::msg::Float64::SharedPtr msg){
                vanttec::CANMessage can_msg;
                for(int i = 0; i<8; i++){
                    vanttec::packFloat(can_msg, 0x15 +i , msg->data);
                    send_frame(0x123, can_msg);
                    RCLCPP_INFO(this->get_logger(), "Received: %f",msg->data);
                }

            }
        );


        individualMotor_sub = this->create_subscription<std_msgs::msg::UInt8>(
            "/uuv/individualMotor/teleop", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg){
                vanttec::CANMessage can_msg;
                int n = msg->data;
                if(n>=7) n = 7;
                if(n<=0) n = 0;
                for(int i = 0; i<8; i++){
                    if(i==n){
                        vanttec::packFloat(can_msg, 0x15 +i , 0.1);
                    } else {
                        vanttec::packFloat(can_msg, 0x15 +i , 0.0);
                    }
                    send_frame(0x123, can_msg);
                }

            }
        );

        twist_teleop_sub = this->create_subscription<geometry_msgs::msg::Twist>("/uuv/cmd_twist", 10, std::bind(&CanNodeUUV::update_motors_callback, this, _1));

        // Initializing Matrixes
        coef = Eigen::MatrixXd::Zero(6, 6);
        thrust = Eigen::VectorXd::Zero(6);

        coef(0,0) =  0.3660; coef(0,1) = -1.0276; coef(0,2) =  0.0; coef(0,3) =     0.0; coef(0,4) = 5.1737; coef(0,5) = -0.9673;
        coef(1,0) =  0.3660; coef(1,1) =  1.0276; coef(1,2) =  0.0; coef(1,3) =     0.0; coef(1,4) = 5.1737; coef(1,5) =  0.9673;
        coef(2,0) = -0.1849; coef(2,1) = -1.0276; coef(2,2) =  0.0; coef(2,3) =     0.0; coef(2,4) = 5.1737; coef(2,5) =  0.9673;
        coef(3,0) = -0.1849; coef(3,1) =  1.0276; coef(3,2) =  0.0; coef(3,3) =     0.0; coef(3,4) = 5.1737; coef(3,5) = -0.9673;
        coef(4,0) = -0.1426; coef(4,1) =  0.0367; coef(4,2) = -0.5; coef(4,3) = -2.0973; coef(4,4) =    0.0; coef(4,5) =  0.4929;
        coef(5,0) = -0.1426; coef(5,1) =  0.0367; coef(5,2) = -0.5; coef(5,3) =  2.0973; coef(5,4) =    0.0; coef(5,5) = -0.4929;
    }



protected:
    void parse_frame(const struct can_frame &frame) override {
        vanttec::CANMessage msg;
        std::copy(std::begin(frame.data), std::end(frame.data), std::begin(msg.data));
        msg.len = frame.can_dlc;
        uint8_t vttec_msg_id = vanttec::getId(msg);
        uint32_t can_id = frame.can_id;
        
        auto steady_clock = rclcpp::Clock();

        // RCLCPP_INFO(this->get_logger(), "Got message from: %#X  with vttec id: %#X", can_id, vttec_msg_id);

        if(can_id == 0x111){
            if(vttec_msg_id == 0x03){
                std_msgs::msg::Float64 encoder_msg;
                encoder_msg.data  = vanttec::getFloat(msg);
                auto steady_clock = rclcpp::Clock();
                // RCLCPP_WARN_THROTTLE(this->get_logger(), steady_clock, 1000, "Got encoder message: %f", encoder_msg.data);
                // steering_angle_pub->publish(encoder_msg);
            }
        }
    }

    
private:
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr allMotor_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr individualMotor_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_teleop_sub;

    // Parameters
    rclcpp::Parameter up_rightMotor_param; rclcpp::Parameter up_leftMotor_param; rclcpp::Parameter rightMotor_param;
    rclcpp::Parameter leftMotor_param; rclcpp::Parameter low_rightMotor_param; rclcpp::Parameter low_leftMotor_param;
    rclcpp::Parameter scaleControl_param;

    // Variables for Calculating Torques
    Eigen::MatrixXd coef;
    Eigen::VectorXd thrust;

    // Motor variables
    int up_rightMotor; int up_leftMotor; int rightMotor; int leftMotor; int low_rightMotor; int low_leftMotor;
    float scaleControl;

    //Callbacks
    void update_motors_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        Eigen::VectorXd t(6);

        t(0)= msg->linear.x; t(1)=msg->linear.y; t(2)=msg->linear.z; t(3)=msg->angular.x; t(4)=msg->angular.y; t(5)=msg->angular.z;

        thrust = coef * t * scaleControl;

        for(int i=0; i<6; i++){
           thrust(i) = thrust(i) > 0.0 ? 0.1 : 0.0;
           thrust(i) = thrust(i) < 0.0 ? -0.1 : 0.0;
        }
        
        //RCLCPP_INFO(this->get_logger(),"%f, %f, %f, %f, %f, %f", msg->linear.x,msg->linear.y,msg->linear.z,msg->angular.x,msg->angular.y,msg->angular.z);
        vanttec::CANMessage can_msg;

        for(int i = 0; i<8; i++){
            if(i < 6){
                if(i == up_rightMotor){
                    vanttec::packFloat(can_msg, 0x15 +  up_rightMotor, thrust(0));
                }
                if(i == up_leftMotor){
                    vanttec::packFloat(can_msg, 0x15 +  up_leftMotor, thrust(1));
                }
                if(i == low_rightMotor){
                    vanttec::packFloat(can_msg, 0x15 +  low_rightMotor, thrust(2));
                }
                if(i == low_leftMotor){
                    vanttec::packFloat(can_msg, 0x15 +  low_leftMotor, thrust(3));
                }
                if(i == rightMotor){
                    vanttec::packFloat(can_msg, 0x15 +  rightMotor, thrust(4));
                }
                if(i == leftMotor){
                    vanttec::packFloat(can_msg, 0x15 +  leftMotor, thrust(5));
                }
            }else{
                vanttec::packFloat(can_msg, 0x15 +i , 0.0);
            }

            send_frame(0x123, can_msg);
        }

        RCLCPP_INFO(this->get_logger(),"%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, s:%.2f", thrust(0), thrust(1), thrust(2), thrust(3), thrust(4), thrust(5), scaleControl);
    }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNodeUUV>());
  rclcpp::shutdown();
  return 0;
}
