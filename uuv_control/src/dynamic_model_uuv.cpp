#include <tf2/LinearMath/Quaternion.h>

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
#include "src/model/uuv_dynamic_model.h"

using namespace std::chrono_literals;

class DynamicModelSim : public rclcpp::Node {
 public:
  DynamicModelSim() : Node("dynamic_model_uuv") {
    using namespace std::placeholders;

    // Declare and acquire `subname` parameter
    subname_ = this->declare_parameter<std::string>("subname", "uuv");

    posePub =
        this->create_publisher<geometry_msgs::msg::Pose>("uuv/state/pose", 10);
    odomPub =
        this->create_publisher<nav_msgs::msg::Odometry>("output/odom", 10);

    thrusterSub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "uuv/forces", 10,
        [this](const std_msgs::msg::Float64MultiArray &msg) {
            RCLCPP_ERROR(this->get_logger(), "ex: %f", 
            msg.data[0]);
            for(int i = 0 ; i < 6 ; i++){
                thruster_input[i] = msg.data[i];
            }
        });

    pose_path_pub = this->create_publisher<nav_msgs::msg::Path>(
        "uuv/pose_path", 10);

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_stamped_tmp_.header.frame_id = "world";
    pose_path.header.frame_id = "world";
    pose_path.header.stamp = DynamicModelSim::now();

    updateTimer = this->create_wall_timer(
        10ms, std::bind(&DynamicModelSim::update, this));
  }

 protected:

  double normalize_angle(double ang){
    double out = std::fmod(ang + M_PI, M_PI*2);
    if(out < 0)
      out+=M_PI*2;
    return out - M_PI;
  }

  void update() {
    model.update(thruster_input);
    UUVState out = model.state;

    /**
     * Output stage
     */
    double x = out.eta[0];  // position in x
    double y = out.eta[1];  // position in y
    double z = out.eta[2];  // position in y
    double roll = normalize_angle(out.eta[3]);  // position in y
    double pitch = normalize_angle(out.eta[4]);  // position in y
    double yaw = normalize_angle(out.eta[5]);  // position in y

    geometry_msgs::msg::Pose pose;
    nav_msgs::msg::Odometry odom;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    tf2::Quaternion qq;
    qq.setRPY(roll, pitch, yaw);

    pose.orientation.w = qq.getW();
    pose.orientation.x = qq.getX();
    pose.orientation.y = qq.getY();
    pose.orientation.z = qq.getZ();
    //  = q[0];
    // pose.orientation.y = q[1];
    // pose.orientation.z = q[2];
    // pose.orientation.w = q[3];

    odom.pose.pose = pose;

    posePub->publish(pose);

    double u, v, w, p, q, r;

    u = out.nu[0];  // surge velocity
    v = out.nu[1];  // sway velocity
    w = out.nu[2];  // heave velocity
    p = out.nu[3];  // roll rate
    q = out.nu[4];  // pitch rate
    r = out.nu[5];  // yaw rate

    odom.twist.twist.linear.x = u;
    odom.twist.twist.linear.y = v;
    odom.twist.twist.linear.z = w;

    odom.twist.twist.angular.x = p;
    odom.twist.twist.angular.y = q;
    odom.twist.twist.angular.z = r;

    pose_stamped_tmp_.pose = pose;
    pose_path.poses.push_back(pose_stamped_tmp_);

    // Erase elements when path is too long
    if(pose_path.poses.size() > 1000*5){
      pose_path.poses.erase(pose_path.poses.begin(), pose_path.poses.begin()+1);
    }
    odom.header = pose_stamped_tmp_.header;
    odomPub->publish(odom);
    pose_path_pub->publish(pose_path);

    tf_broadcast(pose);
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr posePub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_path_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
  rclcpp::TimerBase::SharedPtr updateTimer;

  geometry_msgs::msg::PoseStamped pose_stamped_tmp_;
    nav_msgs::msg::Path pose_path;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr thrusterSub;
  std::array<double, 6> thruster_input;

  UUVDynamicModel model;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::string subname_;

  void tf_broadcast(const geometry_msgs::msg::Pose &msg) {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = subname_.c_str();;

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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicModelSim>());
  rclcpp::shutdown();
  return 0;
}