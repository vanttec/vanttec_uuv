#include "keyboard.hpp"

#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "vehicle_user_control");
  ros::NodeHandle n("~");
  ros::Rate r(50);

  ros::Publisher pub_down = n.advertise<vehicle_user_control::KeyboardKey>("/vehicle_user_control/vehicle_user_control/kb_keydown", 10);
  ros::Publisher pub_up = n.advertise<vehicle_user_control::KeyboardKey>("/vehicle_user_control/vehicle_user_control/kb_keyup", 10);

  bool allow_repeat = false;
  int repeat_delay, repeat_interval;
  
  n.param<bool>("allow_repeat", allow_repeat, false );
  n.param<int>("repeat_delay", repeat_delay, SDL_DEFAULT_REPEAT_DELAY );
  n.param<int>("repeat_interval", repeat_interval, SDL_DEFAULT_REPEAT_INTERVAL );
  
  if (!allow_repeat)
  {
    repeat_delay = 0;
  }
  
  VehicleUserControl::Keyboard keyboard(repeat_delay, repeat_interval);
  vehicle_user_control::KeyboardKey k;

  bool pressed, new_event;
  while (ros::ok() && keyboard.get_key(new_event, pressed, k.code, k.modifiers)) 
  {
    if (new_event) 
    {
      k.header.stamp = ros::Time::now();
      if (pressed) 
      {
        pub_down.publish(k);
      }
      else 
      {
        pub_up.publish(k);
      }
    }
    ros::spinOnce();
    r.sleep();
  }
  ros::waitForShutdown();
}
