
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using std::placeholders::_1;

class PosicionNode : public rclcpp::Node
{
public:
    PosicionNode()
    : Node("posicion"), x_(0.0), y_(0.0), z_(0.0)
    {
        // Publicador de la posición del submarino
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("uuv/state/pose", 10);

        // Suscriptor de la salida del controlador PID
        control_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "uuv/control_output", 10, std::bind(&PosicionNode::control_callback, this, _1));

        // Publica la posición inicial en (0,0,0)
        publish_position();
    }

private:
    void control_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3) {
            RCLCPP_WARN(get_logger(), "Datos insuficientes en control_output");
            return;
        }

        // Integra la velocidad de salida del PID para actualizar la posición
        x_ += msg->data[0] * 0.01; // Pequeño paso de integración
        y_ += msg->data[1] * 0.01;
        z_ += msg->data[2] * 0.01;

        publish_position();
    }

    void publish_position()
    {
        auto pose = geometry_msgs::msg::Pose();
        pose.position.x = x_;
        pose.position.y = y_;
        pose.position.z = z_;

        publisher_->publish(pose);
        RCLCPP_INFO(get_logger(), "Nueva posición: [%.2f, %.2f, %.2f]", x_, y_, z_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr control_subscriber_;

    double x_, y_, z_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PosicionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
