#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Temporal node for bridge between IMU and uuv_control.
// And to handling of uuv_localization

class uuv_localization_node : public rclcpp::Node
{
public:
    uuv_localization_node() : Node("uuv_localization_node")
    {
        // Subscriber
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "uuv/state/pose", 10,
            std::bind(&uuv_localization_node::pose_callback, this, std::placeholders::_1));

        // Publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("vectornav/imu", 10);
        
        // Temporal logger message
        RCLCPP_INFO(this->get_logger(), "Pose to IMU bridge node started");
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr pose_msg)
    {
        sensor_msgs::msg::Imu imu_msg;

        // Simple conversion: copy orientation
        imu_msg.orientation = pose_msg->orientation;

        // Populate header
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        // NOTE: Maybe, this is not useful, so it apply a set to zero. 
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;

        imu_msg.linear_acceleration.x = 0.0;
        imu_msg.linear_acceleration.y = 0.0;
        imu_msg.linear_acceleration.z = 0.0;

        imu_pub_->publish(imu_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uuv_localization_node>());
    rclcpp::shutdown();
    return 0;
}
