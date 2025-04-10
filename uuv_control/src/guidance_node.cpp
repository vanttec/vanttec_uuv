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

using namespace std::chrono_literals;

class GuidanceNode : public rclcpp::Node {
 public:
  GuidanceNode() : Node("guidance_node") {
    using namespace std::placeholders;

    pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "uuv/state/pose", 10,
      [this](const geometry_msgs::msg::Pose &msg) {
        uuv_pose = msg;
      });

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "uuv/path_to_follow", 10,
      [this](const nav_msgs::msg::Path &msg) {
        if(msg.poses.size() != path_poses.size()){
          path_poses = msg.poses;
          wp_index = 0;
          uuv_setpoint = path_poses[wp_index].pose;
          setpoint_pub_->publish(uuv_setpoint);
        }
      });

    setpoint_pub_ =
      this->create_publisher<geometry_msgs::msg::Pose>("uuv/pose_setpoint", 10);

    updateTimer = this->create_wall_timer(
        100ms, std::bind(&GuidanceNode::update, this));

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

 protected:

  void update() {
    if(path_poses.size() == 0)
      return;

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

    double error = std::sqrt(
      std::pow(error_t.transform.translation.x,2)+
      std::pow(error_t.transform.translation.y,2)+
      std::pow(error_t.transform.translation.z,2)
    );

    if(error < 1.0 && wp_index < path_poses.size()){
      wp_index++;
      uuv_setpoint = path_poses[wp_index].pose;
      setpoint_pub_->publish(uuv_setpoint);
    }


  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr setpoint_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::TimerBase::SharedPtr updateTimer;

  geometry_msgs::msg::Pose uuv_pose, uuv_setpoint;

  std::vector<geometry_msgs::msg::PoseStamped> path_poses;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  int wp_index{-1};

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GuidanceNode>());
  rclcpp::shutdown();
  return 0;
}