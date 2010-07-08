// copyright 2010 morgan quigley, mquigley@cs.stanford.edu 
// bsd license blah blah

#include "ros/ros.h"
#include "openarms/ArmSensors.h"
#include "openarms/ArmActuators.h"

openarms::ArmSensors sensors_msg;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "armbus_sim");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("arm_actuators", 1, actuators_cb);
  ros::Publisher pub = n.advertise<openarms::ArmSensors>("arm_sensors",1);
  ros::Time t_prev = ros::Time::now();
  
  sensors_msg.pos.resize(7);

  while (n.ok())
  {
    ros::Duration(0.001).sleep();
    ros::spinOnce();
    // blast status-request packets periodically
    ros::Time t = ros::Time::now();
    if ((t - t_prev).toSec() > 0.05)
    {
      replied = false;
      enable_led(11, even ? 1 : 0);
      even = !even;
      //send_stepper_vel(11, 0x7fff, 0x7fff);
      if (scheduled == SCH_STEPPER_POS_0)
        query_stepper_positions(11);
      else if (scheduled == SCH_STEPPER_POS_1)
        query_stepper_positions(10);
      /*
      else if (scheduled == SCH_STEPPER_POS_1)
        query_accelerometer(11);
        */
      if (++scheduled > SCH_END)
        scheduled = SCH_BEGIN;
      t_prev = t;

#if 0
      for (size_t i = 0; i < 1 /*NUM_MOTORS*/; i++)
        query_motor(i);
#endif
    }
  }
  enable_motor(10, 0); // turn em off
  enable_motor(11, 0); // turn em off
  delete s;
  return 0;

  return 0;
}
