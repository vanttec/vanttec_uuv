/** ----------------------------------------------------------------------------
 * @file: vtec_u4_6dof_dynamic_model.hpp
 * @date: March 20, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of VTec U4 6dof dynamic model in the non-inertial frame
           with Euler Angles.
 * -----------------------------------------------------------------------------
 **/

#ifndef __VTEC_U4_6DOF_DYNAMIC_MODEL__
#define __VTEC_U4_6DOF_DYNAMIC_MODEL__

#include "generic_6dof_uuv_dynamic_model.hpp"

class VTecU4DynamicModel : public Generic6DOFUUVDynamicModel
{
    public:
        VTecU4DynamicModel(float _sample_time_s);
        ~VTecU4DynamicModel();

        void UpdateThrust();
    private:
        Eigen::MatrixXf L;          // Thrust mapping matrix
        Eigen::VectorXf Thrust;     // Thrust vector (6 elements, 1 per thruster)

        /* Thruster configuration parameters */

        float beta;
        float var_epsilon;
        float alpha;
        float delta;
        float gamma;
        float rh_x;
        float rh_y;
        float rh_z;
        float rv_x;
        float rv_y;
        float rv_z;
};

#endif