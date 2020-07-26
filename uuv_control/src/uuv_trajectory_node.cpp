#include <math.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

const float SAMPLE_TIME_S = 0.01;
const float PI            = 3.14159;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uuv_trajectory_node");
    ros::NodeHandle nh;
    
    ros::Rate           cycle_rate(int(1 / SAMPLE_TIME_S));
    ros::Publisher      uuv_setpoints = nh.advertise<geometry_msgs::Twist>("/uuv_control/uuv_control_node/setpoint", 1000);

    geometry_msgs::Twist setpoints;

    int sample_counter = 0;

    while(ros::ok())
    {
        /* Run Queued Callbacks */ 
        ros::spinOnce();

        setpoints.linear.x = 0.25;
        setpoints.linear.y = 0; 
        //setpoints.linear.z = 0.5; 
        //setpoints.linear.y = 0.5 * PI * cos(0.2 * PI * (sample_counter * SAMPLE_TIME_S * setpoints.linear.x));
        setpoints.linear.z = 0.5 * PI * sin(0.2 * PI * (sample_counter * SAMPLE_TIME_S * setpoints.linear.x));;
        setpoints.angular.z = 0;
        sample_counter++;

        /* Publish Odometry */ 
        uuv_setpoints.publish(setpoints);

        /* Slee for 10ms */
        cycle_rate.sleep();
    }
    
    return 0;
}