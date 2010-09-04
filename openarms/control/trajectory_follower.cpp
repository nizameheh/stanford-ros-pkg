// copyright 2010 morgan quigley bsd license blah blah
#include <cstdlib>
#include <cstdio>
#include <termios.h>
#include <ros/ros.h>
#include "openarms/Trajectory.h"
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <cstring>
using std::vector;
using std::string;

struct termios cooked, raw; // mmm makes me hungry
openarms::Trajectory g_traj, g_next_traj;
sensor_msgs::JointState g_js;
bool g_next_traj_valid = false;
enum js_latch_state_t 
{
  JS_LATCH_IDLE, 
  JS_LATCH_WAITING, 
  JS_LATCH_DONE
} js_latch_state = JS_LATCH_IDLE;

void traj_cb(const openarms::Trajectory::ConstPtr traj)
{
  ROS_INFO("received %d-state trajectory", (int)traj->pts.size());
  g_next_traj = *traj;
  g_next_traj_valid = true; // handshake to avoid race condition
}

void js_cb(const sensor_msgs::JointState::ConstPtr js)
{
  if (js_latch_state == JS_LATCH_WAITING)
  {
    g_js = *js;
    js_latch_state = JS_LATCH_DONE;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_follower");
  vector<sensor_msgs::JointState> js_vec;
  vector<string> js_name_vec;

  ros::NodeHandle n;
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("target_joints",
                                                               1);
  ros::Subscriber traj_sub = n.subscribe<openarms::Trajectory>("trajectory", 
                                                               1, traj_cb);
  ros::Subscriber js_sub = n.subscribe<sensor_msgs::JointState>("joint_states",
                                                                1, js_cb);

  tcgetattr(0, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VTIME] = 0;
  raw.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &raw);
  puts("greetings. ctrl-c to exit.");
  ros::Rate rate(50);
  char c;
  ros::Time state_start_t = ros::Time::now();
  enum { RUNNING, STOPPED } controller_state = STOPPED;
  int target_idx = 0;

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
    ros::Time t = ros::Time::now();

    ssize_t nread = read(0, &c, 1);
    if (nread < 0)
    {
      perror("read():");
      exit(-1);
    }
    else if (nread == 0)
      c = 0;
    if (c == ' ')
      controller_state = STOPPED;

    if (g_next_traj_valid)
    {
      g_next_traj_valid = false;
      g_traj = g_next_traj;
      target_idx = 0;
      state_start_t = t;
      controller_state = RUNNING;
      js_latch_state = JS_LATCH_WAITING; // grab the current arm position
    }

    if (controller_state == RUNNING &&
        js_latch_state == JS_LATCH_DONE &&
        target_idx < (int)g_traj.pts.size())
    {
      sensor_msgs::JointState js_start, js_end, js_cmd;
      if (target_idx <= 0)
        js_start = g_js; // latched at the start of the trajectory
      else
        js_start = g_traj.pts[target_idx-1].js; 
      openarms::TrajectoryPoint *pt = &g_traj.pts[target_idx];
      js_end = pt->js;
      // generate joint state command using the requested interpolation
      double state_time = (t - state_start_t).toSec();
      if (pt->interpolation == openarms::TrajectoryPoint::INTERP_NONE)
        js_cmd = js_end;
      else if (pt->interpolation == openarms::TrajectoryPoint::INTERP_LINEAR)
      {
        if (pt->move_sec < 0.001) // sanitize it...
          pt->move_sec = 0.001;
        double x = state_time / pt->move_sec; // in [0,1] for interpolation
        //ROS_INFO("target=%d x=%f", target_idx, x);
        if (x < 0)
          x = 0; // shouldn't happen...
        if (x < 1)
        {
          js_cmd = js_end; // populate the joint names and such
          for (size_t j = 0; j < js_end.position.size(); j++) // convex comb.
            js_cmd.position[j] = (1.0-x) * js_start.position[j] +
                                      x  * js_end.position[j];
        }
        else // we're in the dwell time. hang out here.
          js_cmd = js_end;
      }
      js_pub.publish(js_cmd);
      if (state_time > pt->move_sec + pt->dwell_sec) // shall we move on ?
      {
        target_idx++;
        state_start_t = t;
        if (target_idx >= (int)g_traj.pts.size())
        {
          ROS_INFO("trajectory complete");
          controller_state = STOPPED;
        }
        else
          ROS_INFO("going to state %d", target_idx);
      }
    }
  }
  tcsetattr(0, TCSADRAIN, &cooked);
  printf("bai\n");
  fflush(0);
  return 0;
}

