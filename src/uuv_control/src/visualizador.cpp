#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

class MarkerNode : public rclcpp::Node
{
public:
  MarkerNode() : Node("visualizador")
  {
    posicion_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualizador", 10);
    position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "uuv/control_output", 10, std::bind(&MarkerNode::positionCallback, this, std::placeholders::_1));

    posicion_.header.frame_id = "sub";
    posicion_.ns = "posicion_namespace";
    posicion_.id = 0;
    posicion_.type = visualization_msgs::msg::Marker::SPHERE;
    posicion_.action = visualization_msgs::msg::Marker::ADD;
    posicion_.scale.x = 1.0;
    posicion_.scale.y = 1.0;
    posicion_.scale.z = 1.0;
    posicion_.color.a = 1.0;
    posicion_.color.r = 0.0;
    posicion_.color.g = 1.0;
    posicion_.color.b = 0.0;
  }

private:
  void positionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    posicion_.header.stamp = this->get_clock()->now();
    posicion_.pose.position.x = msg->x;
    posicion_.pose.position.y = msg->y;
    posicion_.pose.position.z = msg->z;
    posicion_pub_->publish(posicion_);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr posicion_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_sub_;
  visualization_msgs::msg::Marker posicion_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarkerNode>());
  rclcpp::shutdown();
  return 0;
}
