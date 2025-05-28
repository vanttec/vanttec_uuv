//  El submarino debe de hundirse 1 metro y hacer un cuadrado de 2x2 metros

// Se subscribe al mensage de posicion de la imu para saber cuando cambiar de waypoint

// Publica una posicion en :

//     "uuv/desired_pose"


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int32.hpp"


#include <vector>

// #include "model/checks.cpp"

class ObjetivoNode : public rclcpp::Node
{
public:
    ObjetivoNode() : Node("objetivo_cuadrado")
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("uuv/desired_pose", 10);
        pose_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "uuv/estado", 10, 
            std::bind(&ObjetivoNode::publish_goal, this, std::placeholders::_1));

            init_poses();
    }

    

private:

    std::vector<geometry_msgs::msg::Pose> dp;
    geometry_msgs::msg::Pose pose;
    
    int counter = 0;
    float error = 0.5;

    void init_poses(){
        geometry_msgs::msg::Pose posed;
        
        posed.position.x = 0.0;
        posed.position.y = 0.0;
        posed.position.z = -1.0;
        dp.push_back(posed);

        posed.position.x = 2.0;
        posed.position.y = 0.0;
        posed.position.z = -1.0;
        dp.push_back(posed);

        posed.position.x = 2.0;
        posed.position.y = 2.0;
        posed.position.z = -1.0;
        dp.push_back(posed);

        posed.position.x = 0.0;
        posed.position.y = 2.0;
        posed.position.z = -1.0;
        dp.push_back(posed);
    }

    void publish_goal(const std_msgs::msg::Int32::SharedPtr state)
    {
        if (state->data == -1){
            pose.position.x = 0.0;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
        }
        else{
        const auto& dp_actual = dp[state->data];

        pose.position.x = dp_actual.position.x;
        pose.position.y = dp_actual.position.y;
        pose.position.z = dp_actual.position.z;
        }
        pose_pub_->publish(pose);
        RCLCPP_INFO(get_logger(), "Publicando posición objetivo: [%.2f, %.2f, %.2f]",
                    pose.position.x, pose.position.y, pose.position.z);
    }


    

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pose_sub_;
    // rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjetivoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}