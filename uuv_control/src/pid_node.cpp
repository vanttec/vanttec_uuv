#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cstdio>
#include <cmath>
#include <algorithm>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "pid/pid.cpp"

using namespace std::chrono_literals;

class PidNode : public rclcpp::Node {
 public:
  PidNode() : Node("pid_node") {
    using namespace std::placeholders;

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "uuv/state/pose", 10,
      [this](const geometry_msgs::msg::Pose &msg) {
        uuv_pose = msg;
      });
    
    setpoint_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "uuv/pose_setpoint", 10,
      [this](const geometry_msgs::msg::Pose &msg) {
        for(int i = 0 ; i < 6 ; i++){
          pid[i].clear();
        }
        uuv_setpoint = msg;
      });

    forces_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("uuv/forces", 10);

    updateTimer = this->create_wall_timer(
        10ms, std::bind(&PidNode::update, this));

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    pid_params[0] = PIDParameters{100.0, 10.0, 300.0, 0.01, 36.5, -30.0, false, 1.0};
    pid_params[1] = PIDParameters{50.0, 10.0, 300.0, 0.01, 36.5, -30.0, false, 1.0};
    pid_params[2] = PIDParameters{100.0, 10.0, 300.0, 0.01, 36.5, -30.0, false, 1.0};
    pid_params[3] = PIDParameters{5.0, 1.0, 10.0, 0.01, 36.5, -30.0, false, 1.0};
    pid_params[4] = PIDParameters{5.0, 1.0, 10.0, 0.01, 36.5, -30.0, false, 1.0};
    pid_params[5] = PIDParameters{5.0, 1.0, 10.0, 0.01, 36.5, -30.0, false, 1.0};

    for(int i = 0 ; i < 6 ; i++){
      pid[i] = PID(pid_params[i]);
      thrusters_msg.data.push_back(0.0);
    }

  }

 protected:

  double normalize_angle(double ang){
    double out = std::fmod(ang + M_PI, M_PI*2);
    if(out < 0)
      out+=M_PI*2;
    return out - M_PI;
  }

  void update() {

    tf_broadcast(uuv_setpoint);

    geometry_msgs::msg::TransformStamped error_t;

    try {
        error_t = tf_buffer_->lookupTransform(
            "uuv", "pose_ref",tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "uuv", "pose_ref", ex.what());
        return;
    }

    thrusters_msg.data[0] = pid[0].update(0.0, error_t.transform.translation.x);
    thrusters_msg.data[1] = pid[1].update(0.0, error_t.transform.translation.y);
    thrusters_msg.data[2] = pid[2].update(0.0, error_t.transform.translation.z);

    tf2::Quaternion q_pose;
    q_pose.setW(uuv_pose.orientation.w);
    q_pose.setX(uuv_pose.orientation.x);
    q_pose.setY(uuv_pose.orientation.y);
    q_pose.setZ(uuv_pose.orientation.z);
    tf2::Matrix3x3 m_pose;
    m_pose.setRotation(q_pose);

    tf2::Quaternion q_setpoint;
    q_setpoint.setW(uuv_setpoint.orientation.w);
    q_setpoint.setX(uuv_setpoint.orientation.x);
    q_setpoint.setY(uuv_setpoint.orientation.y);
    q_setpoint.setZ(uuv_setpoint.orientation.z);
    tf2::Matrix3x3 m_setpoint;
    m_setpoint.setRotation(q_setpoint);

    double roll_pose, pitch_pose, yaw_pose;
    m_pose.getRPY(roll_pose, pitch_pose, yaw_pose);

    double roll_setpoint, pitch_setpoint, yaw_setpoint;
    m_setpoint.getRPY(roll_setpoint, pitch_setpoint, yaw_setpoint);

    double roll_error = normalize_angle(roll_setpoint - roll_pose);
    double pitch_error = normalize_angle(pitch_setpoint - pitch_pose);
    double yaw_error = normalize_angle(yaw_setpoint - yaw_pose);
    
    thrusters_msg.data[3] = pid[3].update(0.0, roll_error);
    thrusters_msg.data[4] = pid[4].update(0.0, pitch_error);
    thrusters_msg.data[5] = pid[5].update(0.0, yaw_error);

    forces_pub_->publish(thrusters_msg);
  }

  void tf_broadcast(const geometry_msgs::msg::Pose &msg) {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "pose_ref";

    t.transform.translation.x = msg.position.x;
    t.transform.translation.y = msg.position.y;
    t.transform.translation.z = msg.position.z;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    t.transform.rotation.w = msg.orientation.w;
    t.transform.rotation.x = msg.orientation.x;
    t.transform.rotation.y = msg.orientation.y;
    t.transform.rotation.z = msg.orientation.z;

    // Send the transformation
    tf_broadcaster->sendTransform(t);
  }

 private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forces_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_, setpoint_sub_;

  rclcpp::TimerBase::SharedPtr updateTimer;

  geometry_msgs::msg::Pose uuv_pose, uuv_setpoint;
  std_msgs::msg::Float64MultiArray thrusters_msg;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::array<double, 6> thrusters_output{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  };

  std::array<PID,6> pid;
  std::array<PIDParameters,6> pid_params;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidNode>());
  rclcpp::shutdown();
  return 0;
}