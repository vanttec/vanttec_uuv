#include "asmc_guidance_4dof.hpp"

ASMC_GUIDANCE_4DOF::ASMC_GUIDANCE_4DOF(double _sample_time_s, const double _Ka,  const double _K2, const double _Kalpha, const double _Kmin, const double _miu)
                                        : asmc_guidance_surge(_sample_time_s,_Ka,_K2,_Kalpha,_Kmin,_miu,LINEAR_DOF )
                                        , asmc_guidance_sway (_sample_time_s,_Ka,_K2,_Kalpha,_Kmin,_miu,LINEAR_DOF )
                                        , asmc_guidance_heave(_sample_time_s,_Ka,_K2,_Kalpha,_Kmin,_miu,LINEAR_DOF )
                                        , asmc_guidance_yaw  (_sample_time_s,_Ka,_K2,_Kalpha,_Kmin,_miu,ANGULAR_DOF){}

ASMC_GUIDANCE_4DOF::~ASMC_GUIDANCE_4DOF(){}

void ASMC_GUIDANCE_4DOF::SetSetpoints(const float set_points[4])
{
    asmc_guidance_surge.asmc.set_point = set_points[0];
    asmc_guidance_sway.asmc.set_point  = set_points[1];
    asmc_guidance_heave.asmc.set_point = set_points[2];
    asmc_guidance_yaw.asmc.set_point   = set_points[3];
}

void ASMC_GUIDANCE_4DOF::CalculateManipulation(const geometry_msgs::Pose& _pose)
{
    Eigen::Vector4d aux;
    G << std::cos(_pose.orientation.z), -std::sin(_pose.orientation.z), 0, 0,
         std::sin(_pose.orientation.z),  std::cos(_pose.orientation.z), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    asmc_guidance_surge.CalculateAuxiliaryControl(_pose.position.x);
    asmc_guidance_sway.CalculateAuxiliaryControl(_pose.position.y);
    asmc_guidance_heave.CalculateAuxiliaryControl(_pose.position.z);
    asmc_guidance_yaw.CalculateAuxiliaryControl(_pose.orientation.z);

    aux <<  asmc_guidance_surge.Uax - asmc_guidance_surge.desired_dot_error -  asmc_guidance_surge.Ka*asmc_guidance_surge.asmc.error,
            asmc_guidance_sway.Uax  - asmc_guidance_sway.desired_dot_error  -  asmc_guidance_sway.Ka*asmc_guidance_sway.asmc.error,
            asmc_guidance_heave.Uax - asmc_guidance_heave.desired_dot_error -  asmc_guidance_heave.Ka*asmc_guidance_heave.asmc.error,
            asmc_guidance_yaw.Uax   - asmc_guidance_yaw.desired_dot_error   -  asmc_guidance_yaw.Ka*asmc_guidance_yaw.asmc.error;
            
    U = -G.inverse()*aux;
}