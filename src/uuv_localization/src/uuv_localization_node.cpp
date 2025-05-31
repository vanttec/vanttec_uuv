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
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "vectornav/imu", 10,
            std::bind(&uuv_localization_node::pose_callback, this, std::placeholders::_1));

        // Publisher
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("uuv/state/pose", 10);
        
        // Temporal logger message
        RCLCPP_INFO(this->get_logger(), "Pose to IMU bridge node started");
    }

private:
    void pose_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {

        geometry_msgs::msg::Pose pose_msg;

        // Copy orientation from IMU
        pose_msg.orientation = imu_msg->orientation;

        // You may estimate or set position here if needed (currently set to zero)
        pose_msg.position.x = 0.0;
        pose_msg.position.y = 0.0;
        pose_msg.position.z = 0.0;

        pose_pub_->publish(pose_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uuv_localization_node>());
    rclcpp::shutdown();
    return 0;
}
