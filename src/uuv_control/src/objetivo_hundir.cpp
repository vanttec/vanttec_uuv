//  El submarino debe de hundirse 1 metro y quedarse en su posicion hundida

// Se subscribe al mensage de posicion de la imu para saber cuando cambiar de waypoint

// Publica una posicion en :

//     "uuv/desired_pose"


//  El submarino debe de hundirse 1 metro y hacer un cuadrado de 2x2 metros

// Se subscribe al mensage de posicion de la imu para saber cuando cambiar de waypoint

// Publica una posicion en :

//     "uuv/desired_pose"


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

// #include "model/checks.cpp"

class ObjetivoNode : public rclcpp::Node
{
public:
    ObjetivoNode() : Node("objetivo_hundir")
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("uuv/desired_pose", 10);

        updateTimer = this->create_wall_timer(1000ms, std::bind(&ObjetivoNode::update, this));

    }

    

private:

    //std::vector<geometry_msgs::msg::Pose> dp;
    geometry_msgs::msg::Pose pose;


    void update(){
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = -1.0;

        pose_pub_->publish(pose);
        RCLCPP_INFO(get_logger(), "Publicando posición objetivo: [%.2f, %.2f, %.2f]",
                    pose.position.x, pose.position.y, pose.position.z);
    }
    

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;

    rclcpp::TimerBase::SharedPtr updateTimer;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjetivoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}