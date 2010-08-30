// copyright 2010 morgan quigley bsd license blah blah
#include <cstdlib>
#include <cstdio>
#include <termios.h>
#include <ros/ros.h>
#include <openarms/Trajectory.h>
#include <vector>
#include <string>
#include <cstring>
using std::vector;
using std::string;

struct termios cooked, raw; // mmm makes me hungry

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_sender");
  if (argc != 3)
  {
    ROS_INFO("usage: trajectory_sender WAYPOINTFILE TRAJECTORYFILE");
    return 1;
  }
  FILE *waypoint_file = fopen(argv[1], "r");
  if (!waypoint_file)
  {
    ROS_ERROR("woah there. couldn't open %s", argv[1]);
    return 1;
  }
  vector<openarms::TrajectoryPoint> tp_vec;
  while (!feof(waypoint_file))
  {
    char line[1000];
    if (!fgets(line, sizeof(line), waypoint_file))
      break;
    if (!strlen(line))
      continue;
    openarms::TrajectoryPoint tp;
    tp.js.name.resize(8);
    tp.js.name[0] = "shoulder1";
    tp.js.name[1] = "shoulder2";
    tp.js.name[2] = "shoulder3";
    tp.js.name[3] = "elbow";
    tp.js.name[4] = "wrist1";
    tp.js.name[5] = "wrist2";
    tp.js.name[6] = "wrist3";
    tp.js.name[7] = "gripper";
    tp.js.position.resize(8);
    for (int i = 0; i < 8; i++)
      tp.js.position[i] = 0;
    const char *token = strtok(line, " \n");
    if (!token)
      continue; // blank line ?
    tp.n = string(token);
    token = strtok(NULL, " \n");
    for (int i = 0; token; i++, token = strtok(NULL, " \n"))
      tp.js.position[i] = atof(token);
    printf("%s: ", tp.n.c_str());
    for (int i = 0; i < 8; i++)
      printf(" %+06.2f", tp.js.position[i]);
    printf("\n");
    tp_vec.push_back(tp);
  }
  fclose(waypoint_file);
  FILE *traj_file = fopen(argv[2], "r");
  if (!traj_file)
  {
    ROS_ERROR("woah there. couldn't open %s", argv[2]);
    return 1;
  }
  openarms::Trajectory traj;
  while (!feof(traj_file))
  {
    char line[1000];
    if (!fgets(line, sizeof(line), traj_file))
      break;
    if (!strlen(line))
      continue;
    const char *token = strtok(line, " \n");
    if (!token)
      continue; // blank line?
    string wp_name(token);
    token = strtok(NULL, " \n");
    if (!token)
    {
      ROS_ERROR("no move time given for wp named [%s]", wp_name.c_str());
      return 1;
    }
    double move_sec = atof(token), dwell_sec = 0;
    token = strtok(NULL, " \n");
    if (token)
      dwell_sec = atof(token);
    // try to find this waypoint
    bool found = false;
    for(vector<openarms::TrajectoryPoint>::iterator it = tp_vec.begin();
        it != tp_vec.end() && !found; ++it)
    {
      if (wp_name == it->n)
      {
        traj.pts.push_back(*it);
        traj.pts.back().move_sec = move_sec;
        traj.pts.back().dwell_sec = dwell_sec;
        traj.pts.back().interpolation = openarms::TrajectoryPoint::INTERP_LINEAR;
        found = true;
      }
    }
    if (!found)
    {
      ROS_ERROR("couldn't find a waypoint named [%s]", wp_name.c_str());
      return 1;
    }
  }

  for (size_t i = 0; i < traj.pts.size(); i++)
  {
    printf("%s: %f %f\n",
           traj.pts[i].n.c_str(),
           traj.pts[i].move_sec,
           traj.pts[i].dwell_sec);
  }

  ros::NodeHandle n;
  ros::Publisher traj_pub = n.advertise<openarms::Trajectory>("trajectory", 1);

  tcgetattr(0, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VTIME] = 1;
  raw.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &raw);
  ROS_INFO("greetings. ctrl-c to exit. press spacebar to send trajectory.");

  while (ros::ok())
  {
    ros::spinOnce();
    char c;
    ssize_t nread = read(0, &c, 1);
    if (nread < 0)
    {
      perror("read():");
      exit(-1);
    }
    else if (nread == 0)
      c = 0;
    if (c == ' ')
    {
      traj_pub.publish(traj);
      ROS_INFO("sending trajectory");
    }
  }
  tcsetattr(0, TCSADRAIN, &cooked);
  ROS_INFO("bai");
  fflush(0);
  return 0;
}

