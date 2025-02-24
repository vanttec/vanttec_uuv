#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Message types
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "control/pid6dof.h"

class PID6DOFNode : public rclcpp::Node {
public:
    PID6DOFNode() : Node("pid_6dof_node"), 
                    current_state_(Eigen::VectorXf::Zero(6)),
                    desired_state_(Eigen::VectorXf::Zero(6)) {
        
        // Initialize parameters
        auto params = initialize_params();
        controller = std::make_unique<PID6DOF>(params);

        // Subscriptions
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "uuv/state/pose", 10, 
            std::bind(&PID6DOFNode::pose_callback, this, std::placeholders::_1));

        desired_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "uuv/desired_pose", 10,
            std::bind(&PID6DOFNode::desired_pose_callback, this, std::placeholders::_1));

        // Publishers
        control_output_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "uuv/control_output", 10);

        // Setup update timer
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100 Hz update rate
            std::bind(&PID6DOFNode::update, this));
    }

private:
    // Controller instance
    std::unique_ptr<PID6DOF> controller;
    
    // State vectors
    Eigen::VectorXf current_state_;
    Eigen::VectorXf desired_state_;

    // ROS Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desired_pose_sub_;

    // ROS Publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_output_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    std::array<PIDParams, 6> initialize_params() {
        // Degrees of freedom
        std::vector<std::string> dof_labels = {"x", "y", "z", "roll", "pitch", "yaw"};
        
        // Get default parameters
        auto default_params = PID::defaultParams();
        
        // Create parameter map
        std::map<std::string, double> params_map;
        for (const auto& dof : dof_labels) {
            declare_parameter("kp_" + dof, default_params.kP);
            declare_parameter("ki_" + dof, default_params.kI);
            declare_parameter("kd_" + dof, default_params.kD);
            declare_parameter("dt_" + dof, default_params.dt);
            declare_parameter("u_max_" + dof, default_params.kUMax);
            declare_parameter("u_min_" + dof, default_params.kUMin);
            declare_parameter("enable_ramp_" + dof, default_params.enable_ramp_rate_limit);
            declare_parameter("ramp_rate_" + dof, default_params.ramp_rate);
        }

        // Read parameters
        std::array<PIDParams, 6> params;
        for (size_t i = 0; i < 6; i++) {
            const auto& dof = dof_labels[i];
            params[i] = default_params;
            params[i].kP = get_parameter("kp_" + dof).as_double();
            params[i].kI = get_parameter("ki_" + dof).as_double();
            params[i].kD = get_parameter("kd_" + dof).as_double();
            params[i].dt = get_parameter("dt_" + dof).as_double();
            params[i].kUMax = get_parameter("u_max_" + dof).as_double();
            params[i].kUMin = get_parameter("u_min_" + dof).as_double();
            params[i].enable_ramp_rate_limit = get_parameter("enable_ramp_" + dof).as_bool();
            params[i].ramp_rate = get_parameter("ramp_rate_" + dof).as_double();
            
            RCLCPP_INFO(get_logger(), 
                       "PID [%s] -> Kp: %.2f, Ki: %.2f, Kd: %.2f", 
                       dof.c_str(), params[i].kP, params[i].kI, params[i].kD);
        }
        return params;
    }

    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        current_state_[0] = msg->position.x;
        current_state_[1] = msg->position.y;
        current_state_[2] = msg->position.z;

        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        current_state_[3] = roll;
        current_state_[4] = pitch;
        current_state_[5] = yaw;
    }

    void desired_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        desired_state_[0] = msg->position.x;
        desired_state_[1] = msg->position.y;
        desired_state_[2] = msg->position.z;

        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        desired_state_[3] = roll;
        desired_state_[4] = pitch;
        desired_state_[5] = yaw;
    }

    void update() {
        if (!controller) {
            RCLCPP_WARN(get_logger(), "Controller not initialized!");
            return;
        }

        auto control_outputs = controller->update(current_state_ , desired_state_);
        
        auto output_msg = std_msgs::msg::Float64MultiArray();
        output_msg.data.resize(6);
        for (int i = 0; i < 6; ++i) {
            output_msg.data[i] = control_outputs[i];
        }
        
        control_output_pub_->publish(output_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PID6DOFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}