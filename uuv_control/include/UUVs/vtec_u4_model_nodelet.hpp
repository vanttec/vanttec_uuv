/** ----------------------------------------------------------------------------
 * @file: vtec_u4_model_nodelet.hpp
 * @date: August 30, 2022
 * @author: Sebastian Mtz
 * @email: sebas.martp@gmail.com
 * 
 * @brief: VTec U4 model nodelet. Does not currently work.
 * -----------------------------------------------------------------------------
**/

#ifndef __VTEC_U4_MODEL_NODELET_H__
#define __VTEC_U4_MODEL_NODELET_H__

#include "generic_6dof_uuv_dynamic_model.hpp"
#include "vtec_u4_6dof_dynamic_model.hpp"
// #include "vanttec_msgs/EtaPose.h"
#include "vanttec_msgs/SystemDynamics.h"
#include <std_msgs/MultiArrayDimension.h>

#include <ros/ros.h>    
// #include <stdio.h>
#include <nodelet/nodelet.h>

class VTecU4ModelNodelet : public nodelet::Nodelet
{
    private:
        virtual void onInit();

    public:

    VTecU4ModelNodelet();
    ~VTecU4ModelNodelet();
};

#endif