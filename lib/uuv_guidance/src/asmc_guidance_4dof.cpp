/** ----------------------------------------------------------------------------
 * @file: asmc_guidance_4dof.cpp
 * @date: June 20, 2021
 * @author: Carlos Medina
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: 4DOF ASMC Guidance Class
 * -----------------------------------------------------------------------------
 * */

#include "asmc_guidance_4dof.hpp"

ASMC_GUIDANCE_4DOF::ASMC_GUIDANCE_4DOF(double _sample_time_s, const double _Ka,  const double _K2, const double _Kalpha, const double _Kmin, const double _miu)
                                        : asmc_guidance_surge(_sample_time_s,_Ka,_K2,_Kalpha,_Kmin,_miu,LINEAR_DOF )
                                        , asmc_guidance_sway (_sample_time_s,_Ka,_K2,_Kalpha,_Kmin,_miu,LINEAR_DOF )
                                        , asmc_guidance_heave(_sample_time_s,_Ka,_K2,_Kalpha,_Kmin,_miu,LINEAR_DOF )
                                        , asmc_guidance_yaw  (_sample_time_s,_Ka,_K2,_Kalpha,_Kmin,_miu,ANGULAR_DOF){}

ASMC_GUIDANCE_4DOF::~ASMC_GUIDANCE_4DOF(){}

void ASMC_GUIDANCE_4DOF::SetSetpoints(const float set_points[4])
{
    asmc_guidance_surge.set_point = set_points[0];
    asmc_guidance_sway.set_point  = set_points[1];
    asmc_guidance_heave.set_point = set_points[2];
    asmc_guidance_yaw.set_point   = set_points[3];
    asmc_guidance_surge.Reset();
    asmc_guidance_sway.Reset();
    asmc_guidance_heave.Reset();
    asmc_guidance_yaw.Reset();
}

void ASMC_GUIDANCE_4DOF::CalculateManipulation(const geometry_msgs::Pose& _pose)
{
    Eigen::Vector4d aux;
    G << std::cos(_pose.orientation.z), -std::sin(_pose.orientation.z), 0, 0,
         std::sin(_pose.orientation.z),  std::cos(_pose.orientation.z), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    asmc_guidance_surge.Manipulation(_pose.position.x);
    asmc_guidance_sway.Manipulation(_pose.position.y);
    asmc_guidance_heave.Manipulation(_pose.position.z);
    asmc_guidance_yaw.Manipulation(_pose.orientation.z);

    aux <<  asmc_guidance_surge.Uax - asmc_guidance_surge.desired_dot_error -  asmc_guidance_surge.Ka*asmc_guidance_surge.error,
            asmc_guidance_sway.Uax  - asmc_guidance_sway.desired_dot_error  -  asmc_guidance_sway.Ka*asmc_guidance_sway.error,
            asmc_guidance_heave.Uax - asmc_guidance_heave.desired_dot_error -  asmc_guidance_heave.Ka*asmc_guidance_heave.error,
            asmc_guidance_yaw.Uax   - asmc_guidance_yaw.desired_dot_error   -  asmc_guidance_yaw.Ka*asmc_guidance_yaw.error;
            
    U = -G.inverse()*aux;
}