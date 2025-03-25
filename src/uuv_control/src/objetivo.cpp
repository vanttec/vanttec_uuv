
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

class ObjetivoNode : public rclcpp::Node
{
public:
    ObjetivoNode() : Node("objetivo")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("uuv/desired_pose", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&ObjetivoNode::publish_goal, this));
    }

private:
    void publish_goal()
    {
        auto pose = geometry_msgs::msg::Pose();

        // Configurar la posición objetivo
        pose.position.x = 10.0;
        pose.position.y = 5.0;
        pose.position.z = -3.0;

        publisher_->publish(pose);
        RCLCPP_INFO(get_logger(), "Publicando posición objetivo: [%.2f, %.2f, %.2f]",
                    pose.position.x, pose.position.y, pose.position.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjetivoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
