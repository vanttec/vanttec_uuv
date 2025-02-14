/*
Por hacer:
 - Checar las librerias (todos los includes)
 - Checar tipo de variables (se pelean entre las librerias que son double, float, etc)
 - checar el update
 - Darle chatgptazo

Codigo original:
 Sebastian Martinez sebas.martp@gmail.com creado el 6 de mayo de 2023 en ROS

Codigo actualizado:
 Alberto Serrano chinoalbertoeugenio@gmail.com creado el 12 de enero de 2023 
 (este wey no sabe programar ROS2, asi que chequen bien lo que haga :P)
*/


#include <algorithm>
#include <cmath>
#include <cstdio>

#include "control/PID.h" //Checar la libreria


#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class PIDNode : public rclcpp::Node{
    public:
    PIDNode() : Node("PID"){
        using namespace std::placeholders;
        params = initialize_params();
        controller = PID(params);

// Subscripciones ==================================================================================
        trajectory_sub = this->create_subscription<vanttec_msgs::msg::EtaPose>( //Checar el mensaje
            "/uuv_motion_planning/trajectory_planner/s_curve", 10,
            [this](const vanttec_msgs::msg:EtaPose::SharedPtr msg){
                model->updateTrajectoryReference(*msg);
            });
// Publicaciones ============================================================================
        accel_pub = this->create_publisher<geometry_msgs::msg::Vector3>("/vectornav/ins_3d/ind_acc", 10); // No se como dejo la accel abraham
        vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/uuv_simulation/dynamic_model/vel", 10);
        eta_pose_pub = this->create_publisher<vanttec_msgs::msg::etaPose>("/uuv_simulation/dynamic_model/eta_pose", 10);
        thrust_pub = this->create_publisher<vanttec_msgs::msg::ThrustControl>("/uuv_control/uuv_control_node/thrust", 10);
// Variables =================================================================================
        this->declare_parameter<std::vector<double>>("k_p", {1,1,1,1,1,1});
        this->declare_parameter<std::vector<double>>("k_i", {0,0,0,0,0,0});
        this->declare_parameter<std::vector<double>>("k_d", {0,0,0,0,0,0});
        this->declare_parameter<std::vector<double>>("init_pose", {0,0,0,0,0,0});

        auto k_p_param = this->get_parameter("k_p").as_double_array();
        auto k_i_param = this->get_parameter("k_i").as_double_array();
        auto k_d_param = this->get_parameter("k_d").as_double_array();
        auto init_pose_param = this->get_parameter("init_pose").as_double_array();

        std::array<float, 6> k_p = {k_p_param[0], k_p_param[1], k_p_param[2], k_p_param[3], k_p_param[4], k_p_param[5]};
        std::array<float, 6> k_i = {k_i_param[0], k_i_param[1], k_i_param[2], k_i_param[3], k_i_param[4], k_i_param[5]};
        std::array<float, 6> k_d = {k_d_param[0], k_d_param[1], k_d_param[2], k_d_param[3], k_d_param[4], k_d_param[5]};
        std::array<float, 6> init_pose = {init_pose_param[0], init_pose_param[1], init_pose_param[2],
                                          init_pose_param[3], init_pose_param[4], init_pose_param[5]};
// Iniziar PID ====================================================================================
        std::array<float, 6> max_tau{127, 34, 118, 28, 9.6, 36.6};
        std::array<DOFControllerType_E, 6> types{LINEAR_DOF, LINEAR_DOF, LINEAR_DOF, ANGULAR_DOF, ANGULAR_DOF, ANGULAR_DOF};
        model = std::make_unique<VTEC_U4_6DOF_PID>(SAMPLE_TIME_S, k_p, k_i, k_d, max_tau, types);

        model->setInitPose(init_pose);

        updateTimer = 
        this->create_wall_timer(10ms, std::bind(&PIDNode::controlLoop, this));
    }

    private:
    void controlLoop() {
// Actualizar los estados del modelo ================================================================
        model->calculateStates();
        model->updateNonLinearFunctions();
        model->updateCurrentReference();
        model->calculateControlSignals();
        model->updateControlSignals();

// Publicar los datos =========================================================================
        accel_pub->publish(model->accelerations_);
        vel_pub->publish(model->velocities_);
        eta_pose_pub->publish(model->eta_pose_);
        // uuv_thrust_pub->publish(model->thrust_); // Si se implementa thrust_
    }

    std::vector<float> k_p, k_i, k_d, init_pose;
    std::unique_ptr<VTEC_U4_6DOF_PID> model;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accel_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Publisher<vanttec_msgs::msg::EtaPose>::SharedPtr eta_pose_pub;
    rclcpp::Publisher<vanttec_msgs::msg::ThrustControl>::SharedPtr thrust_pub;

    rclcpp::Subscription<vanttec_msgs::msg::EtaPose>::SharedPtr trajectory_sub;

    rclcpp::TimerBase::SharedPtr updateTimer;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDNode>());
    rclcpp::shutdown();
    return 0;
}
