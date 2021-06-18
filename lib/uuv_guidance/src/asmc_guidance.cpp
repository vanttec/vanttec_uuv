#include "asmc_guidance.hpp"

ASMC_GUIDANCE::ASMC_GUIDANCE(double _sample_time_s, const double _Ka,  const double _K2, const double _Kalpha, const double _Kmin, const double _miu, const DOFControllerType_E _type)
                            :asmc(_sample_time_s,_K2,_Kalpha,_Kmin,_miu,_type)
{
    Ka = _Ka;
    error_i = 0;
    U = 0;
    Uax = 0;
    desired_dot_error = 0;
}

ASMC_GUIDANCE::~ASMC_GUIDANCE(){}

void ASMC_GUIDANCE::CalculateAuxiliaryControl(double _current)
{
    asmc.prev_error = asmc.error;
    asmc.prev_dot_error = asmc.dot_error;
    asmc.error = asmc.set_point - _current;
    asmc.dot_error = asmc.error/asmc.sample_time_s;

    error_i += asmc.error*asmc.sample_time_s;

    if (asmc.controller_type == ANGULAR_DOF)
    {
        if (std::abs(asmc.error) > PI)
        {
            asmc.error = (asmc.error / std::abs(asmc.error)) * (std::abs(asmc.error) - 2 * PI);
        }
        if (std::abs(asmc.dot_error) > PI)
        {
            asmc.dot_error = (asmc.dot_error / std::abs(asmc.dot_error)) * (std::abs(asmc.dot_error) - 2 * PI);
        }
    }

    asmc.sigma = asmc.error + Ka*error_i;
    double sign = (std::abs(asmc.sigma) - asmc.miu) / (std::abs(asmc.sigma) - asmc.miu);

    double dot_K1 = asmc.K1>asmc.Kmin ?  asmc.Kalpha*sign:asmc.Kmin;
    asmc.K1 += dot_K1*asmc.sample_time_s;

    Uax = -asmc.K1*std::sqrt(std::abs(asmc.sigma))*sign - asmc.K2*asmc.sigma;
}