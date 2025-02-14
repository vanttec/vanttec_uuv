#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "vn/ezasyncdata.h"
#include "vn/thread.h"

#include <iostream>


using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;


int main(int argc, char **argv)
{

  //ros initiatilization
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 1000);
  ros::Rate loop_rate(10);

  //imu initiatilization
  // First determine which port your sensor is attached to and update the
  // constant below. Also, if you have changed your sensor from the factory
  // default baudrate of 115200, you will need to update the baudrate
  // constant below as well.
  const string SensorPort = "/dev/ttyUSB0"; // Linux format for virtual (USB) serial port.
  const uint32_t SensorBaudrate = 115200;

  EzAsyncData* ez= EzAsyncData::connect(SensorPort, SensorBaudrate);

  while (ros::ok())
  {
    CompositeData cd = ez->getNextData();
    if (!cd.hasYawPitchRoll())
    {
      ROS_INFO("YPR Unavaible %s", cd.yawPitchRoll());
    }
    else
    {
      vec3f data = cd.yawPitchRoll();
      std_msgs::Float32MultiArray msg;
      msg.data.clear();
      msg.data.push_back(data[0]);
      msg.data.push_back(data[1]);
      msg.data.push_back(data[2]);
      ROS_INFO("YPR %f %f %f", data[0], data[1], data[2]);
      chatter_pub.publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete ez;

  return 0;
}
