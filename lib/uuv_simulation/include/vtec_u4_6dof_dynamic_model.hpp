/** ----------------------------------------------------------------------------
 * @file: vtec_u4_6dof_dynamic_model.hpp
 * @date: March 20, 2022
 * @author: Sebas Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Description of VTec U4 6dof dynamic model.
 * -----------------------------------------------------------------------------
 **/

 #ifndef __VTEC_U4_6DOF_DYNAMIC_MODEL__
 #define __VTEC_U4_6DOF_DYNAMIC_MODEL__

 #include "generic_6dof_uuv_dynamic_model.h"
 #include "vtec_u4_parameters.h"
 
 class VTecU4DynamicModel : public Generic6DOFUUVDynamicModel
 {
    public:
        VTecU4DynamicModel(float _sample_time_s);
        ~VTecU4DynamicModel();
    private:
 }

 #endif