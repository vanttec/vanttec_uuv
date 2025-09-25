// Este nodo recibe la salida de control del pid y la convierte en la accion de los motores
// https://docs.google.com/document/d/1J46TvV1WbtOj7GB3-SL5VAXwkTM1FQy_/edit?usp=sharing&ouid=118337403111481747146&rtpof=true&sd=true

// Se subscribe a:

// "uuv/control_output"

// Hay que checar que es lo que recibe la stm32. En teoria lo que ocupamos es un pwm entre 0 y 255 (sin contar que esta limitado al 50%)
// En teoria no publica nada, sino que puede enviar la senal por can a la stm32.
// Dependiendo de como funcione lo de can, sera si ocupa un nodo para enviar la senal o lo hace directamente


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;

class MatrizLocacion : public rclcpp::Node
{
public: 
    MatrizLocacion() : Node("matriz_locacion")
    {
        pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/uuv/matriz_locacion", 10);
        sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/uuv/forces", 10, std::bind(&MatrizLocacion::read, this, std::placeholders::_1)
        );

        // init_poses();

    }

    

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

   

    void read(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received [%2f, %2f, %2f, %2f, %2f, %2f]", 
            msg->data[0], msg->data[1], msg->data[2], msg->data[3], 
            msg->data[4], msg->data[5]);

        float x = msg->data[0];
        float y = msg->data[1];
        float z = msg->data[2];
        float pitch = msg->data[3];
        float roll = msg->data[4];
        float yaw = msg->data[5]; 

        // float alpha = 75.92;
        // float phi = 24.81;
        // float gamma = 105.00;
        // float beta = 15.00;
        // float epsilon = 20.00;
        // float theta = 24.81;

        float r_x1 = 1.0, r_y1 = 0.5, r_z1 = 0.2, r_y2 = 0.7;

        auto deg2rad = [](double deg){ return deg * M_PI / 180.0; };
        float ct = std::cos(deg2rad(24.81));
        float ca = std::cos(deg2rad(75.92));
        float cy = std::cos(deg2rad(105.00));
        float u = r_y1*cy+r_z1*ca;
        float v = -r_x1*cy+r_z1*ct;
        float w = r_x1*ca+r_y1*ct;

        

        float magnitude = 0;
        for (int i=0; i<6; i++) { magnitude += std::pow(msg->data[i],2); }
        magnitude = std::sqrt(magnitude);

        RCLCPP_INFO(this->get_logger(), "Magnitude %.2f", magnitude);

        Eigen::Matrix<double,6,6> A;
        A << ct, ct, -ct, -ct, 0, 0,
             -ca, ca, -ca, ca, 0, 0,
             -cy, -cy, cy, cy, -1, -1,
             u, -u, -u, u, -r_y2, r_y2,
             v, v, v, v, 0, 0,
             -w, w, w, -w, 0, 0;

        Eigen::Matrix<double,6,1> tau;
        if (magnitude != 0) {
            tau << x/magnitude, y/magnitude, z/magnitude, roll/magnitude, pitch/magnitude, yaw/magnitude;
        }
        else tau << x, y, z, roll, pitch, yaw;
        

        // Resolver sistema matricial
        Eigen::Matrix<double,6,1> F = A.colPivHouseholderQr().solve(tau);

        RCLCPP_INFO(this->get_logger(), "Computed [%.2f, %.2f, %2.f, %.2f, %.2f, %.2f]", 
            F[0], F[1], F[2], F[3], F[4], F[5]);

        std_msgs::msg::Float64MultiArray toPublish;
        toPublish.data.resize(6);
        
        for (int i=0; i<6; i++) {
            toPublish.data[i] = F[i];
        }
        
        pub_->publish(toPublish);
        
    }

    


};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MatrizLocacion>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

