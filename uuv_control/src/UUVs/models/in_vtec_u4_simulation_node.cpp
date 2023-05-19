/** ----------------------------------------------------------------------------
 * @file: in_vtec_u4_simulation_node.cpp
 * @date: April 10, 2022
 * @author: Sebas Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: ROS simulation node for the VTec U-IV with intertial mode..
 * https://answers.ros.org/question/190920/how-can-i-subscribe-to-a-topic-using-a-parent-class-function-as-the-callback/
 * @todo: Check if the nonlinear functions publishing section should be here or in
 * the VTecU4DynamicModel class.
 * -----------------------------------------------------------------------------
 **/

#include "dynamic_models/marine_vehicles/underwater/vtec_u4_in_6dof_dynamic_model.hpp"
#include "vanttec_msgs/EtaPose.h"
#include "vanttec_msgs/SystemDynamics.h"
#include <std_msgs/MultiArrayDimension.h>

#include <ros/ros.h>
#include <stdio.h>

static const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inert_vtec_u4_simulation_node");
    ros::NodeHandle private_nh("~");
        
    ros::Rate               cycle_rate(int(1 / SAMPLE_TIME_S));
    VTecU4InDynamicModel      uuv_model(SAMPLE_TIME_S);
    vanttec_msgs::SystemDynamics  uuv_functions;

    std::vector<float> init_pose;
    private_nh.param("init_pose", init_pose, {0,0,0});
    
    ros::Publisher  uuv_accel     = private_nh.advertise<geometry_msgs::Vector3>("/vectornav/ins_3d/ins_acc", 10);
    ros::Publisher  uuv_vel       = private_nh.advertise<geometry_msgs::Twist>("/uuv_simulation/dynamic_model/vel", 10);
    ros::Publisher  uuv_eta_pose_  = private_nh.advertise<vanttec_msgs::EtaPose>("/uuv_simulation/dynamic_model/eta_pose", 10);
    ros::Publisher  uuv_dynamics  = private_nh.advertise<vanttec_msgs::SystemDynamics>("/uuv_simulation/dynamic_model/non_linear_functions", 10);

    ros::Subscriber uuv_thrust_input = private_nh.subscribe("/uuv_control/uuv_control_node/thrust", 
                                                    10, 
                                                    &Marine6DOFInDynamicModel::thrustCallbacK,
                                                    dynamic_cast<Marine6DOFInDynamicModel*> (&uuv_model));
            
    uuv_functions.g.layout.dim.push_back(std_msgs::MultiArrayDimension());
    uuv_functions.g.layout.dim.push_back(std_msgs::MultiArrayDimension());
    uuv_functions.g.layout.dim[0].label = "rows";
    uuv_functions.g.layout.dim[1].label = "cols";
    uuv_functions.g.layout.dim[0].size = 6;
    uuv_functions.g.layout.dim[1].size = 6;
    uuv_functions.g.layout.dim[0].stride = 6;
    uuv_functions.g.layout.data_offset = 0;

    uuv_model.setInitPose(init_pose);
    while(ros::ok())
    {
        /* Run Queued Callbacks */
        ros::spinOnce();

        /* calculate Model States */
        uuv_model.calculateStates();

        /* Publish Odometry */
        uuv_accel.publish(uuv_model.accelerations_);
        uuv_vel.publish(uuv_model.velocities_);
        uuv_eta_pose_.publish(uuv_model.eta_pose_);
        
        /* Publish nonlinear functions */

        // uuv_functions.f = {uuv_model.f_(0), uuv_model.f_(1), uuv_model.f_(2), uuv_model.f_(3), uuv_model.f_(4), uuv_model.f_(5)};
        // uuv_functions.g.data = { uuv_model.g_(0,0), uuv_model.g_(0,1), uuv_model.g_(0,2), uuv_model.g_(0,3), uuv_model.g_(0,4), uuv_model.g_(0,5),
        //                          uuv_model.g_(1,0), uuv_model.g_(1,1), uuv_model.g_(1,2), uuv_model.g_(1,3), uuv_model.g_(1,4), uuv_model.g_(1,5),
        //                          uuv_model.g_(2,0), uuv_model.g_(2,1), uuv_model.g_(2,2), uuv_model.g_(2,3), uuv_model.g_(2,4), uuv_model.g_(2,5),
        //                          uuv_model.g_(3,0), uuv_model.g_(3,1), uuv_model.g_(3,2), uuv_model.g_(3,3), uuv_model.g_(3,4), uuv_model.g_(3,5),
        //                          uuv_model.g_(4,0), uuv_model.g_(4,1), uuv_model.g_(4,2), uuv_model.g_(4,3), uuv_model.g_(4,4), uuv_model.g_(4,5),
        //                          uuv_model.g_(5,0), uuv_model.g_(5,1), uuv_model.g_(5,2), uuv_model.g_(5,3), uuv_model.g_(5,4), uuv_model.g_(5,5) };
        
        uuv_dynamics.publish(uuv_functions);

        /* Sleep for 10ms */
        cycle_rate.sleep();
    }

    return 0;
}