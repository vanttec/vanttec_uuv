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

#ifndef __UUV_ASMC_GUIDANCE_4DOF_H__
#define __UUV_ASMC_GUIDANCE_4DOF_H__

#include "asmc_guidance.hpp"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>

class ASMC_GUIDANCE_4DOF
{      
    public:
        Eigen::Vector4d U;
        Eigen::Matrix4d G;

        ASMC_GUIDANCE asmc_guidance_surge;
        ASMC_GUIDANCE asmc_guidance_sway;
        ASMC_GUIDANCE asmc_guidance_heave;
        ASMC_GUIDANCE asmc_guidance_yaw;

        // Constructor
        ASMC_GUIDANCE_4DOF(double _sample_time_s, const double _Ka,  const double _K2, const double _Kalpha, const double _Kmin, const double _miu);

        // Destructor
        ~ASMC_GUIDANCE_4DOF();

        void CalculateManipulation(const geometry_msgs::Pose& _pose);
        void SetSetpoints(const float set_points[4]);
};

#endif