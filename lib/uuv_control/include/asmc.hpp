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
        float sample_time_s;

        double set_point;
        double error;
        double prev_error;
        double dot_error;
        double prev_dot_error;

        double manipulation;

        double lambda;
        double sigma;           //Sliding surface

        double K1;
        double K2;
        double Kmin;
        double Kalpha;
        double miu;

        DOFControllerType_E controller_type;

        // Constructor
        ASMC(double _sample_time_s, const double _K2, const double _Kalpha, const double _Kmin, const double _miu, const DOFControllerType_E _type);

        // Destructor
        ~ASMC();

        void SetAdaptiveParams(const double _Kmin, const double _Kalpha, const double _miu);
        void Manipulation(double);
};

#endif