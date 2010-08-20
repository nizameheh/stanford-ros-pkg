#include <ros/ros.h>
#include "openarms/ArmSensors.h"
#include "openarms/ArmActuators.h"
using std::string;

double g_gripper_target = 0;
ros::Publisher *g_actuator_pub = NULL;
openarms::ArmActuators g_actuators;
//double g_stepper_vel[4];
uint16_t g_encoder_target[4] = {9000, 0, 0, 0};

void sensors_cb(const openarms::ArmSensors::ConstPtr &msg)
{
  const double MAX_VEL_BIGDOGS = 3000, MAX_VEL_LITTLEDOGS = 3000;
  const double MAX_ACCEL_PER_SEC = 30000;
  static ros::Time s_prev_time;
  static bool s_prev_time_init = false;
  static double s_prev_encoder[4];

  if (!s_prev_time_init)
  {
    s_prev_time_init = true;
    s_prev_time = ros::Time::now();
    for (int i = 0; i < 4; i++)
      s_prev_encoder[i] = 0;
    return;
  }
  else
  {
    ros::Time t = ros::Time::now();
    double dt = (t - s_prev_time).toSec();
    s_prev_time = t;
    const int32_t MAX_ACCEL = (int32_t)(MAX_ACCEL_PER_SEC * dt);
    double vel[4];
    for (int i = 0; i < 4; i++)
    {
      vel[i] = msg->encoder[i] - s_prev_encoder[i];
      s_prev_encoder[i] = msg->encoder[i];
      if (vel[i] < -8000)
        vel[i] += 10000;
      else if (vel[i] > 8000)
        vel[i] -= 10000;
    }
    
    //printf("vel[0] = %.0f\n", vel[0]);

    for (int i = 0; i < 4; i++)
    {
      /*
      if (i > 0)
      {
        g_actuators.stepper_vel[i] = 0;
        continue;
      }
      */
      double pos_err = (double)g_encoder_target[i] - (double)msg->encoder[i];
      if (pos_err < -5000)
        pos_err += 10000;
      else if (pos_err > 5000)
        pos_err -= 10000;

      double desired_vel = 0;
      if (i == 0)
        desired_vel = 97*vel[0];
      else if (i == 1)
      {
        desired_vel = 0.8096*97.0*vel[0];//50*vel[0] + 150*(vel[1] + 0.25*vel[0]);
        //desired_vel = 80*vel;
      }
      else if (i == 2)
        desired_vel = 100*vel[2];
      else if (i == 3)
        desired_vel = 100*vel[3];

      //double desired_vel = 50*vel; //1*pos_err + 1*(0 - vel);

      /*
      if (err < -10)
        desired_vel = -100;
      else if (err > 10)
        desired_vel = 100;
      */
      double accel = desired_vel - g_actuators.stepper_vel[i];
      // enforce acceleration limit
      if (accel > MAX_ACCEL)
        accel = MAX_ACCEL;
      else if (accel < -MAX_ACCEL)
        accel = -MAX_ACCEL;
      // enforce velocity limit
      g_actuators.stepper_vel[i] += accel;
      const double MAX_VEL = (i < 2 ? MAX_VEL_BIGDOGS : MAX_VEL_LITTLEDOGS);
      if (g_actuators.stepper_vel[i] > MAX_VEL)
        g_actuators.stepper_vel[i] = MAX_VEL;
      else if (g_actuators.stepper_vel[i] < -MAX_VEL)
        g_actuators.stepper_vel[i] = -MAX_VEL;

      /*
      printf("vel = %.0f,  desired = %.0f, cmd = %d\n",
             vel, desired_vel, g_actuators.stepper_vel[i]);
      */
      /*
         if (abs(g_actuators.stepper_vel[i]) < 20)
         g_actuators.stepper_vel[i] = 0;
       */
      /*
         if (abs(g_actuators.stepper_vel[i]) < 15)
         {
         if (g_actuators.stepper_vel[i] > 0)
         g_actuators.stepper_vel[i] = 10;
         else
         g_actuators.stepper_vel[i] = -10;
         }
       */
    }
    for (int i = 4; i < 8; i++)
    {
      g_actuators.servo_torque[i-4] = 0;
      /*
      double err = 0;
      err = g_target.position[i] - state_msg->position[i];
      double desired_torque = 1000 * err;
      // enforce torque limit
      int32_t MAX_TORQUE = 500;
      if (desired_torque > MAX_TORQUE)
        desired_torque = MAX_TORQUE;
      else if (desired_torque < -MAX_TORQUE)
        desired_torque = -MAX_TORQUE;
      g_actuators.servo_torque[i-4] = desired_torque;
      */
    }
    g_actuator_pub->publish(g_actuators);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_stepper_home");
  ros::NodeHandle n;
  ros::Subscriber sensors_sub = n.subscribe("arm_sensors", 1, sensors_cb);
  ros::Publisher actuator_pub = n.advertise<openarms::ArmActuators>("arm_actuators_autopilot", 1);

  g_actuator_pub = &actuator_pub;
  g_actuators.stepper_vel.resize(4);
  g_actuators.servo_torque.resize(4);
  for (int i = 0; i < 4; i++)
  {
    g_actuators.stepper_vel[i] = 0;
    g_actuators.servo_torque[i] = 0;
  }

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  for (int i = 0; i < 4; i++)
  {
    g_actuators.stepper_vel[i] = 0;
    g_actuators.servo_torque[i] = 0;
  }
  g_actuator_pub->publish(g_actuators);
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  return 0;
}

