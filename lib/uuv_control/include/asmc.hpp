/** ----------------------------------------------------------------------------
 * @file: asmc.hpp
 * @date: June 17, 2021
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Adaptive Sliding Mode Controller class, which implements a single DOF controller.
 * -----------------------------------------------------------------------------
 * */

#ifndef __ASMC_H__
#define __ASMC_H__

#include <cmath>
#define PI 3.14159265359

typedef enum DOFControllerType_E
{
    LINEAR_DOF = 0,
    ANGULAR_DOF = 1,
} DOFControllerType_E;

class ASMC
{
    public:
        double set_point;
        double error1;
        double prev_error1;
        double error2;
        double prev_error2;

        double ua;

        double lambda;
        double s;           //Sliding surface

        double K1;
        double dot_K1;
        double prev_dot_K1;
        double K2;
        double K_min;
        double K_alpha;
        double mu;

        DOFControllerType_E controller_type;

        // Constructor
        ASMC(const double _K2, const double _K_alpha, const double _K_min, const double _mu, const DOFControllerType_E _type);

        // Destructor
        ~ASMC();

        void Reset();
        void SetAdaptiveParams(const double _K_min, const double _K_alpha, const double _mu);
        void CalculateAuxControl(double set_point, double _current_pos, double _current_vel, double _sample_time_s);
};

#endif