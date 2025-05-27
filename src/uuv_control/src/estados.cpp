#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int32.hpp"

#include <vector>
#include <cmath>

// #include "model/checks.cpp"

class EstadosNode : public rclcpp::Node
{
public: 
    EstadosNode() : Node("estados")
    {
        pub_ = this->create_publisher<std_msgs::msg::Int32>("uuv/estado", 10);
        sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/uuv/state/pose", 10, std::bind(&EstadosNode::read, this, std::placeholders::_1)
        );

        init_poses();

    }

    

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_;

    std::vector<geometry_msgs::msg::Pose> dp;
    std_msgs::msg::Int32 state;
    float error = 0.1;
    

    void init_poses(){
        state.data = 0;
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
        

        RCLCPP_INFO(this->get_logger(), "Terminado [%2f, %2f, %2f]", 
                dp[state.data].position.x, dp[state.data].position.y, dp[state.data].position.z);


        // Publicar cuando se inicia en 0,0,0
        std_msgs::msg::Int32 first_state;
        first_state.data = -1;
        pub_->publish(first_state);


    }

    void read(const geometry_msgs::msg::Pose::SharedPtr pose){
        RCLCPP_INFO(this->get_logger(), "Received [%2f, %2f, %2f]", 
            pose->position.x, pose->position.y, pose->position.z);
        
        bool change = check(pose, dp, state, error);
        
        if (change){
            state.data = (state.data+1) % dp.size();

            RCLCPP_INFO(this->get_logger(), "En camino al punto [%2f, %2f, %2f]", 
                dp[state.data].position.x, dp[state.data].position.y, dp[state.data].position.z);            
        }

        pub_->publish(state);
    }

    bool check(geometry_msgs::msg::Pose::SharedPtr pose, std::vector<geometry_msgs::msg::Pose> dp, std_msgs::msg::Int32 state, float error){
    
        /*
        state = -1 -> está yendo de 0,0,0 a 0,0,-1 EL VALOR DE STATE DE ESTA FUNCIÓN NO TOMARÁ ESE VALOR
        state = 0 -> está yendo de 0,0,-1 a 2,0,-1
        state = 1 -> está yendo de 2,0,-1 a 2,2,-1
        state = 2 -> está yendo de 2,2,-1 a 0,2,-1
        state = 3 -> está yendo de 0,-2,-1 a 0,0,-1
        */
    
        const auto& dp_actual = dp[state.data];
    
        float dpx = dp_actual.position.x;
        float dpy = dp_actual.position.y;
        float dpz = dp_actual.position.z;
    
        float distance = std::sqrt(std::pow(pose->position.x - dpx, 2) + 
                std::pow(pose->position.y - dpy, 2) + 
                std::pow(pose->position.z - dpz, 2));
        return (distance <= error);
    
    }


};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstadosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}