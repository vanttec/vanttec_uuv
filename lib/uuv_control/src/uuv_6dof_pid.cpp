/** ----------------------------------------------------------------------------
 * @file: uuv_6dof_pid.cpp
 * @date: April 10, 2022
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 *
 * @brief: 6-DOF feedback linearization PID controller class.
 * -----------------------------------------------------------------------------
 * */

#include "uuv_6dof_pid.hpp"

UUV6DOFPIDController::UUV6DOFPIDController(const float &_sample_time_s, const float *_k_p, const float *_k_i, const float *_k_d, const DOFControllerType_E *_type) : 
                    PID_x    (_sample_time_s, _k_p[0], _k_i[0], _k_d[0], _type[0]),
                    PID_y    (_sample_time_s, _k_p[1], _k_i[1], _k_d[1], _type[1]),
                    PID_z    (_sample_time_s, _k_p[2], _k_i[2], _k_d[2], _type[2]),
                    PID_phi  (_sample_time_s, _k_p[3], _k_i[3], _k_d[3], _type[3]),
                    PID_theta(_sample_time_s, _k_p[4], _k_i[4], _k_d[4], _type[4]),
                    PID_psi  (_sample_time_s, _k_p[5], _k_i[5], _k_d[5], _type[5])
{

    u.resize(6,1);              // Control
    ua.resize(6,1);             // Auxiliary Control
    f.resize(6,1);
    g.resize(6,6);
    ref_dot_dot.resize(6,1);

    u << 0,
         0,
         0,
         0,
         0,
         0;

    ua << 0,
         0,
         0,
         0,
         0,
         0;

    ref_dot_dot << 0,
                   0,
                   0,
                   0,
                   0,
                   0;

    g = Eigen::MatrixXf::Zero(6,6);

    f << 0,
         0,
         0,
         0,
         0,
         0;

    thrust.tau_x = 0;
    thrust.tau_y = 0;
    thrust.tau_z = 0;
    thrust.tau_phi = 0;
    thrust.tau_theta = 0;
    thrust.tau_psi = 0;
    
    functs_arrived = 0;
}

UUV6DOFPIDController::~UUV6DOFPIDController(){};


void UUV6DOFPIDController::UpdateSetPoints(const vanttec_uuv::EtaPose& _set_points)
{
    PID_x.UpdateSetPoint(_set_points.x);
    PID_y.UpdateSetPoint(_set_points.y);
    PID_z.UpdateSetPoint(_set_points.z);
    PID_phi.UpdateSetPoint(_set_points.phi);
    PID_theta.UpdateSetPoint(_set_points.theta);
    PID_psi.UpdateSetPoint(_set_points.psi);
}

void UUV6DOFPIDController::UpdatePose(const vanttec_uuv::EtaPose& _current)
{
    PID_x.CalculateManipulation(_current.x);
    PID_y.CalculateManipulation(_current.y);
    PID_z.CalculateManipulation(_current.z);
    PID_phi.CalculateManipulation(_current.phi);
    PID_theta.CalculateManipulation(_current.theta);
    PID_psi.CalculateManipulation(_current.psi);
}

void UUV6DOFPIDController::CalculateManipulations()
{
    if (functs_arrived) {
        ua << PID_x.manipulation,
            PID_y.manipulation,
            PID_z.manipulation,
            PID_phi.manipulation,
            PID_theta.manipulation,
            PID_psi.manipulation;

        u << g.inverse()*(ref_dot_dot - f + ua);

        if(fabs(u(0)) > 127)  u(0) = u(0) / fabs(u(0)) * 127;
        if(fabs(u(1)) > 34)   u(1) = u(1) / fabs(u(1)) * 34;
        if(fabs(u(2)) > 118)  u(2) = u(2) / fabs(u(2)) * 118;
        if(fabs(u(3)) > 28)   u(3) = u(3) / fabs(u(3)) * 28;
        if(fabs(u(4)) > 9.6)  u(4) = u(4) / fabs(u(4)) * 9.6;
        if(fabs(u(5)) > 36.6) u(5) = u(5) / fabs(u(5)) * 36.6;
    
        thrust.tau_x = u(0);
        thrust.tau_y = u(1);
        thrust.tau_z = u(2);
        thrust.tau_phi = u(3);
        thrust.tau_theta = u(4);
        thrust.tau_psi = u(5);
    }
}

void UUV6DOFPIDController::UpdateDynamics(const vanttec_uuv::SystemDynamics& _non_linear_functions)
{
    uint8_t stride =  (uint8_t) _non_linear_functions.g.layout.dim[0].stride;
    uint8_t offset =  (uint8_t) _non_linear_functions.g.layout.data_offset;

    for(int i=0; i<6; ++i){
        for(int j=0; j<6; ++j){
            g(i,j) = _non_linear_functions.g.data[offset + i*stride + j];
        }
    }

    // g << _non_linear_functions.g[0][0], _non_linear_functions.g[0][1], _non_linear_functions.g[0][2], _non_linear_functions.g[0][3], _non_linear_functions.g[0][4], _non_linear_functions.g[0][5],
    //      _non_linear_functions.g[1][0], _non_linear_functions.g[1][1], _non_linear_functions.g[1][2], _non_linear_functions.g[1][3], _non_linear_functions.g[1][4], _non_linear_functions.g[1][5],
    //      _non_linear_functions.g[2][0], _non_linear_functions.g[2][1], _non_linear_functions.g[2][2], _non_linear_functions.g[2][3], _non_linear_functions.g[2][4], _non_linear_functions.g[2][5],
    //      _non_linear_functions.g[3][0], _non_linear_functions.g[3][1], _non_linear_functions.g[3][2], _non_linear_functions.g[3][3], _non_linear_functions.g[3][4], _non_linear_functions.g[3][5],
    //      _non_linear_functions.g[4][0], _non_linear_functions.g[4][1], _non_linear_functions.g[4][2], _non_linear_functions.g[4][3], _non_linear_functions.g[4][4], _non_linear_functions.g[4][5],
    //      _non_linear_functions.g[5][0], _non_linear_functions.g[5][1], _non_linear_functions.g[5][2], _non_linear_functions.g[5][3], _non_linear_functions.g[5][4], _non_linear_functions.g[5][5];

    f << _non_linear_functions.f[0],
         _non_linear_functions.f[1],
         _non_linear_functions.f[2],
         _non_linear_functions.f[3],
         _non_linear_functions.f[4],
         _non_linear_functions.f[5];

    // std::cout << "f:" << f << std::endl;
    // std::cout << "g:" << g << std::endl;

    functs_arrived = 1;
}