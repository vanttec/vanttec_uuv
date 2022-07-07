/** ----------------------------------------------------------------------------
 * @file: asmc.hpp
 * @date: June 17, 2021
 * @date: June 4, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#ifndef __ASMC_H__
#define __ASMC_H__

#include "uuv_common.hpp"
#include "vanttec_uuv/EtaPose.h"
#include <cmath>

class ASMC
{
    private:
        float _sample_time_s;
        float _q_d;         // Setpoint
        float _q_dot_d;
        float _error1;
        float _error2;
        float _prev_error1;
        float _prev_error2;
        
        // Auxiliar control
        float _ua;

        // Sliding surface
        float _lambda;
        float _s;

        // Gains
        float _K1;
        float _K2;
        float _dot_K1;
        float _prev_dot_K1;

        // Adaptive law
        float _K_min;
        float _K_alpha;
        float _mu;

        DOFControllerType_E _controller_type;
    public:
        // Constructor
        ASMC(const float sample_time_s, const float lambda,  const float K2, const float K_alpha, const float K1_init, const float K_min, const float mu, const DOFControllerType_E type);

        // Destructor
        ~ASMC();

        void Reset();
        void UpdateSetPoint(const float q_d, const float q_dot_d);
        void CalculateAuxControl(float q, float q_dot);

        friend class UUV_6DOF_ASMC;
};

#endif