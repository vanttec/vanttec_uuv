#pragma once

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

// Message types
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/bool.hpp>

// Custom headers
#include "control/pid6dof.h" 

class PID6DOFNode : public rclcpp::Node {
public:
PID6DOFNode() : Node("pid_6dof_node") {

  // Initialize parameters of controller
  params = initialize_params();
  controller = PID6DOFController(params);

  //Subscriptions
  pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
    "uuv/state/pose", 10, 
    std::bind(&PID6DOFNode::pose_callback, this, std::placeholders::_1)
  );

  velocity_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "uuv/state/velocity", 10, 
    std::bind(&PID6DOFNode::velocity_callback, this, std::placeholders::_1)
  );

  setpoint_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>(
    "setpoint/pose", 10, 
    std::bind(&PID6DOFNode::setpoint_pose_callback, this, std::placeholders::_1)
  );

  setpoint_velocity_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "setpoint/velocity", 10, 
    std::bind(&PID6DOFNode::setpoint_velocity_callback, this, std::placeholders::_1)
  );

  auto_mode_sub = this->create_subscription<std_msgs::msg::UInt16>(
    "uuv/op_mode", 10, 
    std::bind(&PID6DOFNode::auto_mode_callback, this, std::placeholders::_1)
  );

  // Publishers
  control_output_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "uuv/control_output", 10
  );

  error_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "debug/control_error", 10
  );

  gain_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "debug/control_gains", 10
  );

  // Setup update timer
  update_timer = this->create_wall_timer(
    std::chrono::milliseconds(10),  // 100 Hz update rate
      std::bind(&PID6DOFNode::update, this)
  );
}

private:

  // Controller instance
  PID6DOFController controller;
  PID6DOFParams params;

  // ROS Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr setpoint_pose_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr setpoint_velocity_sub;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr auto_mode_sub;

  // ROS Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_output_pub;
    
  // Debug publishers (max los tenia, asi que los agregue)
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gain_pub;

  // Timer
  rclcpp::TimerBase::SharedPtr update_timer;

  // Current state variables
  State6DOF current_state;
  State6DOF desired_state;
  std_msgs::msg::UInt16 auto_mode;

  // Initialization of parameters
  PID6DOFParams initialize_params(){
    // Define default parameters
    auto default_params = PID6DOFController::defaultParams();
    
    // Create parameter map for ROS2 parameter server
    auto params_map = std::map<std::string, double>({
        {"kP_pos", default_params.kP_pos},
        {"kI_pos", default_params.kI_pos},
        {"kD_pos", default_params.kD_pos},
        {"kP_vel", default_params.kP_vel},
        {"kI_vel", default_params.kI_vel},
        {"kD_vel", default_params.kD_vel},
        {"dt", default_params.dt},
        {"u_max", default_params.u_max},
        {"u_min", default_params.u_min}
    });

    this->declare_parameters("", params_map);

    // Read parameters 
    PID6DOFParams p = default_params;
    p.kP_pos = this->get_parameter("kP_pos").as_double();
    p.kI_pos = this->get_parameter("kI_pos").as_double();
    p.kD_pos = this->get_parameter("kD_pos").as_double();
    p.kP_vel = this->get_parameter("kP_vel").as_double();
    p.kI_vel = this->get_parameter("kI_vel").as_double();
    p.kD_vel = this->get_parameter("kD_vel").as_double();
    p.dt = this->get_parameter("dt").as_double();
    p.u_max = this->get_parameter("u_max").as_double();
    p.u_min = this->get_parameter("u_min").as_double();

    return p;
  }

  // Callback methods
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg){
    current_state.position << msg->position.x, msg->position.y, msg->position.z;
    
    // Convert quaternion to Euler angles
    tf2::Quaternion q(
        msg->orientation.x, 
        msg->orientation.y, 
        msg->orientation.z, 
        msg->orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    current_state.orientation << roll, pitch, yaw;
  }

  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    
    desired_state.position << msg->position.x, msg->position.y, msg->position.z;
    
    // Convert quaternion to Euler angles
    tf2::Quaternion q(
        msg->orientation.x, 
        msg->orientation.y, 
        msg->orientation.z, 
        msg->orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    desired_state.orientation << roll, pitch, yaw;
  }

  void PID6DOFNode::setpoint_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    desired_state.position << msg->position.x, msg->position.y, msg->position.z;
    
    // Convert quaternion to Euler angles
    tf2::Quaternion q(
        msg->orientation.x, 
        msg->orientation.y, 
        msg->orientation.z, 
        msg->orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    desired_state.orientation << roll, pitch, yaw;
  }
  
  void PID6DOFNode::setpoint_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    desired_state.velocity << msg->linear.x, msg->linear.y, msg->linear.z;
    desired_state.angular_velocity << msg->angular.x, msg->angular.y, msg->angular.z;
  }

  void PID6DOFNode::auto_mode_callback(const std_msgs::msg::UInt16::SharedPtr msg) {
    auto_mode = *msg;
}

  void PID6DOFNode::update() {
    if (auto_mode.data == 0) {  // Auto mode is 0 in the usv apparently
        // Compute control outputs
        Eigen::VectorXd control_outputs = controller.update(current_state, desired_state);

        // Publish control outputs
        std_msgs::msg::Float64MultiArray output_msg;
        output_msg.data.resize(6);
        for (int i = 0; i < 6; ++i) {
            output_msg.data[i] = control_outputs(i);
        }
        control_output_pub->publish(output_msg);

        std_msgs::msg::Float64MultiArray error_msg, gain_msg;
        // error_pub->publish(error_msg);
        // gain_pub->publish(gain_msg);
    }h
}
};