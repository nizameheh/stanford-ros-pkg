#include <cstdlib>
#include <cstdio>
#include <termios.h>
#include <ros/ros.h>
#include <vector>
#include "openarms/ArmActuators.h"
#include "openarms/ArmSensors.h"
using std::string;
using std::vector;
using std::pair;
using std::make_pair;

openarms::ArmSensors g_sensors;
ros::Publisher *g_actuator_pub = NULL;
openarms::ArmActuators g_actuators;
typedef pair< double, vector < int > > traj_step_t;
typedef vector< traj_step_t > trajectory_t; 

void sensors_cb(const openarms::ArmSensors::ConstPtr &sensors)
{
  g_sensors = *sensors;
}

void send_vel(int v0, int v1, int v2, int v3, bool print = true)
{
  if (print)
    printf("sending: %8d %8d %8d %8d\n", v0, v1, v2, v3);
  g_actuators.stepper_vel[0] = v0;
  g_actuators.stepper_vel[1] = v1;
  g_actuators.stepper_vel[2] = v2;
  g_actuators.stepper_vel[3] = v3;
  g_actuator_pub->publish(g_actuators);
}

void print_trajectory(trajectory_t traj)
{
  for (trajectory_t::iterator it = traj.begin(); it != traj.end(); ++it)
  {
    printf("%.3f: %d %d %d %d\n",
           it->first,
           it->second[0], it->second[1], it->second[2], it->second[3]);
  }
}

traj_step_t make_traj_step(double delta_t, int v0, int v1, int v2, int v3)
{
  static double s_prev_t = 0;
  traj_step_t step;
  step.first = s_prev_t + delta_t;
  s_prev_t += delta_t;

  step.second.resize(4);
  step.second[0] = v0;
  step.second[1] = v1;
  step.second[2] = v2;
  step.second[3] = v3;
  return step;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relative_control_skeleton");
  ros::NodeHandle n;
  ros::Subscriber sensors_sub = n.subscribe("arm_sensors", 1, sensors_cb);
  g_actuators.stepper_vel.resize(4);
  g_actuators.servo_torque.resize(4);
  for (int i = 0; i < 4; i++)
    g_actuators.stepper_vel[i] = g_actuators.servo_torque[i] = 0; // hai. stop.
  ros::Publisher actuator_pub = n.advertise<openarms::ArmActuators>("arm_actuators_autopilot", 1);
  g_actuator_pub = &actuator_pub;

  // make the trajectory
  trajectory_t traj;
  const int MAX_T = 10;
  traj.push_back(make_traj_step(0.0, 0, 0, 0, 0)); // initial velocity
  for (int i = 0; i < MAX_T; i++)
  {
    traj.push_back(make_traj_step(0.2, 0, 0, 0, 15));
    traj.push_back(make_traj_step(1.0, 0, 0, 0,  0));
  }
  print_trajectory(traj);

  struct termios cooked, raw; // tasty
  tcgetattr(0, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VTIME] = 0;
  raw.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &raw);
  
  ros::Time prev_t = ros::Time::now(), controller_start_t = ros::Time::now();
  ros::Rate rate(100); // 100 hz, whatever, sounds good
  enum { RUNNING, STOPPED } controller_state = STOPPED;
  int target_idx = 0;
  printf("secret keys: 'a' to start controller, 'b' to stop, Ctrl+C to quit\n");

  while (ros::ok())
  {
    rate.sleep(); // phew i'm tired
    
    ros::Time t = ros::Time::now();
    prev_t = t;

    if (controller_state == RUNNING)
    {
      if (target_idx < (int)traj.size() - 1)
      {
        /*
        printf("%f, waiting until %f\n", 
               (t - controller_start_t).toSec(),
               traj[target_idx+1].first);
        */
        if ((t - controller_start_t).toSec() > traj[target_idx+1].first)
        {
          target_idx++;
          send_vel(traj[target_idx].second[0],
                   traj[target_idx].second[1],
                   traj[target_idx].second[2],
                   traj[target_idx].second[3]);
        }
      }
      else
      {
        controller_state = STOPPED;
        target_idx = 0;
      }
    }
    else
      send_vel(0, 0, 0, 0, false); // just because i'm paranoid. STOP STOP STOP
    
    char c;
    ssize_t nread = read(0, &c, 1);    
    if (nread < 0)
    {
      perror("read():");
      exit(1);
    }
    else if (nread == 0)
      continue;
    else if (c == 'a') // A for awesome
    {
      printf("awesum! running controller.\n");
      controller_state = RUNNING;
      target_idx = 0;
      controller_start_t = t;
    }
    else if (c == 'b') // B for bogus
    {
      printf("ok, ok, i'll stop\n");
      send_vel(0, 0, 0, 0);
      controller_state = STOPPED;
    }
    else
      printf("key: %c\n", c);
  }
  tcsetattr(0, TCSADRAIN, &cooked);
  printf("bai\n");
  fflush(0);
  return 0;
}

