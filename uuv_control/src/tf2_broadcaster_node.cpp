/* Creo que esto deferia jalar
  TODO:
  - Revisar que si jale lol ._. -Soni */

#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/Pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("usv_tf2_frame_publisher")
  {
    // Declare and acquire `usv_name` parameter
    usv_name_ = this->declare_parameter<std::string>("uuv_name", "uuv");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // callback function on each message
    std::ostringstream stream;
    stream << "/" << uuv_name_.c_str() << "/state/pose";
    std::string topic_name = stream.str();

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    topic_name, 10,
    std::bind(&FramePublisher::handle_uuv_pose, this, std::placeholders::_1));

  }

void handle_uuv_pose(const std::shared_ptr<geometry_msgs::msg::Pose> msg){
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = submarine_name_; // Updated for clarity

    // Set translation from the Pose message
    t.transform.translation.x = msg->position.x;
    t.transform.translation.y = msg->position.y;
    t.transform.translation.z = msg->position.z;

    // Normalize the quaternion using tf2::Quaternion
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    q.normalize(); 

    // Set rotation from the Pose message
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    // Send the transformation
    tf_broadcaster_->sendTransform(t);


}


  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string uuv_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}