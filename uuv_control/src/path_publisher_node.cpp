#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <algorithm>
#include <cstdio>
#include <chrono>
#include <stack>
#include <cmath>
#include <string>
#include <optional>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <limits>
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class PathPublisherNode : public rclcpp::Node
{
public:
    PathPublisherNode() : Node("path_publisher_node")
    {

        path_to_follow_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/uuv/path_to_follow", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization_marker_array", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&PathPublisherNode::update, this));

        path_to_follow.header.frame_id = "world";

        int num_points = 200;
        double radius = 5.0;
        double z_step = 0.05;
        double theta_step = 0.2;
        
        for (int i = 0; i < num_points; ++i) {
            double theta = i * theta_step;
            double x = radius * std::cos(theta);
            double y = radius * std::sin(theta);
            double z = i * z_step;
        
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_to_follow.header;
            pose_stamped.pose.position.x = x;
            pose_stamped.pose.position.y = y;
            pose_stamped.pose.position.z = z;
        
            // Compute tangential vector (derivative of helix) for orientation
            double dx = -radius * std::sin(theta);
            double dy =  radius * std::cos(theta);
            double dz = z_step / theta_step;
        
            // Normalize tangent vector
            double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
            dx /= norm;
            dy /= norm;
            dz /= norm;
        
            // Use the tangent vector to compute yaw, pitch, roll
            double yaw = std::atan2(dy, dx);
            double pitch = std::atan2(-dz, std::sqrt(dx * dx + dy * dy));
            double roll = 0.0; // No roll since the path doesn’t twist around its axis
        
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();
        
            path_to_follow.poses.push_back(pose_stamped);
        }

        for (size_t i = 0; i < path_to_follow.poses.size(); ++i) {
            const auto& pose_stamped = path_to_follow.poses[i];
        
            visualization_msgs::msg::Marker marker;
            marker.header = pose_stamped.header;
            marker.ns = "waypoints";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose_stamped.pose;
        
            marker.scale.x = 0.5;  // Shaft length
            marker.scale.y = 0.1;  // Shaft diameter
            marker.scale.z = 0.1;  // Head diameter
        
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        
            // marker.lifetime = rclcpp::Duration::from_seconds(0); // Permanent marker
        
            marker_array.markers.push_back(marker);
        }
    }

private:
    rclcpp::TimerBase::SharedPtr timer_{nullptr};

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_to_follow_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    visualization_msgs::msg::MarkerArray marker_array;
    
    nav_msgs::msg::Path path_to_follow;

    void update(){
        path_to_follow_pub_->publish(path_to_follow);
        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisherNode>());
    rclcpp::shutdown();
    return 0;
}