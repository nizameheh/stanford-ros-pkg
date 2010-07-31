// copyright 2010 morgan quigley, mquigley@cs.stanford.edu 
// bsd license blah blah

#include "ros/ros.h"
#include "openarms/ArmSensors.h"
#include "openarms/ArmActuators.h"

openarms::ArmSensors sensors_msg;

int32_t g_stepper_vel[4], g_servo_torque[4];
int g_stepper_dirs[4] = {1, -1, 1, 1};

void actuators_cb(const openarms::ArmActuators::ConstPtr &msg)
{
  if (msg->stepper_vel.size() != 4)
  {
    ROS_INFO("woah I was expecting 4 stepper velocities");
    return;
  }
  if (msg->servo_torque.size() != 4)
  {
    ROS_INFO("ahhh I was expecting 3 servo torques");
    return;
  }
  for (int i = 0; i < 4; i++)
    g_stepper_vel[i] = g_stepper_dirs[i] * msg->stepper_vel[i];
  for (int i = 0; i < 4; i++)
    g_servo_torque[i] = msg->servo_torque[i];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "armbus_sim");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("arm_actuators", 1, actuators_cb);
  ros::Publisher pub = n.advertise<openarms::ArmSensors>("arm_sensors",1);
  ros::Time t_prev = ros::Time::now();

  double stepper_pos[4], servo_pos[4];

  for (int i = 0; i < 4; i++)
    stepper_pos[i] = g_stepper_vel[i] = 0;
  for (int i = 0; i < 4; i++)
    servo_pos[i] = g_servo_torque[i] = 0;

  openarms::ArmSensors sensors_msg;
  sensors_msg.pos.resize(8);

  while (n.ok())
  {
    ros::Duration(0.001).sleep();
    ros::spinOnce();
    // blast status-request packets periodically
    ros::Time t = ros::Time::now();
    if ((t - t_prev).toSec() > 0.05)
    {
      ros::Duration dt(t - t_prev);
      double d = dt.toSec();
      t_prev = t;
      for (int i = 0; i < 4; i++)
      {
        stepper_pos[i] += d * g_stepper_dirs[i] * g_stepper_vel[i];      
        sensors_msg.pos[i] = (int32_t)stepper_pos[i];
      }
      for (int i = 0; i < 4; i++)
      {
        servo_pos[i] += d * 1 * g_servo_torque[i];
        sensors_msg.pos[i+4] = (int32_t)servo_pos[i];
      }
      pub.publish(sensors_msg);
    }
  }
  return 0;
}
