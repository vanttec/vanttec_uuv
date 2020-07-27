#include <ros/ros.h>

const float SAMPLE_TIME_S = 0.01;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_control_node");
    ros::NodeHandle nh;
    
    ros::Rate           cycle_rate(int(1 / SAMPLE_TIME_S));
    UUV4DOFController   system_controller(SAMPLE_TIME_S, Kpid_u, Kpid_v, Kpid_z, Kpid_psi);
    
    ros::Publisher  uuv_thrust      = nh.advertise<uuv_control::ThrustControl>("/uuv_control/uuv_control_node/thrust", 1000);

    ros::Subscriber uuv_pose        = nh.subscribe("/uuv_simulation/dynamic_model/pose",
                                                    1000,
                                                    &UUV4DOFController::UpdatePose,
                                                    &system_controller);

    
    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        /* Update Parameters with new info */ 
        system_controller.UpdateControlLaw();
        system_controller.UpdateThrustOutput();

        /* Publish Odometry */ 
        uuv_thrust.publish(system_controller.thrust);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}