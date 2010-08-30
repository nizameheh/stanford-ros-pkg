// copyright 2010 morgan quigley bsd license blah blah
// some code lifted from the teleop_head_keyboard program by Kevin Watts
#include <cstdlib>
#include <cstdio>
#include <termios.h>
#include <ros/ros.h>
#include <openarms/SetJointTarget.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <cstring>
using std::vector;
using std::string;

struct termios cooked, raw; // mmm makes me hungry

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_keyboard");
  if (argc != 2)
  {
    ROS_INFO("usage: waypoint_keyboard WAYPOINTFILE");
    return 1;
  }
  FILE *f = fopen(argv[1], "r");
  if (!f)
  {
    ROS_ERROR("woah there. couldn't open %s", argv[1]);
    return 1;
  }
  vector<sensor_msgs::JointState> js_vec;
  vector<string> js_name_vec;
  while (!feof(f))
  {
    char line[1000];
    if (!fgets(line, sizeof(line), f))
      break;
    if (!strlen(line))
      continue;
    sensor_msgs::JointState js;
    js.name.resize(8);
    js.name[0] = "shoulder1";
    js.name[1] = "shoulder2";
    js.name[2] = "shoulder3";
    js.name[3] = "elbow";
    js.name[4] = "wrist1";
    js.name[5] = "wrist2";
    js.name[6] = "wrist3";
    js.name[7] = "gripper";
    js.position.resize(8);
    for (int i = 0; i < 8; i++)
      js.position[i] = 0;
    const char *token = strtok(line, " \n");
    if (!token)
      continue; // blank line ?
    js_name_vec.push_back(string(token));
    token = strtok(NULL, " \n");
    for (int i = 0; token; i++, token = strtok(NULL, " \n"))
      js.position[i] = atof(token);
    printf("%c) %s: ", (uint8_t)js_vec.size() + 'a', js_name_vec.back().c_str());
    js_vec.push_back(js);
    for (int i = 0; i < 8; i++)
      printf(" %+06.2f", js.position[i]);
    printf("\n");
  }

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::JointState>("target_joints", 1);

  tcgetattr(0, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VTIME] = 1;
  raw.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &raw);
  puts("greetings. ctrl-c to exit.");

  while (ros::ok())
  {
    char c;
    ssize_t nread = read(0, &c, 1);
    if (nread < 0)
    {
      perror("read():");
      exit(-1);
    }
    else if (nread == 0)
      c = 0;
    if (c >= 'a' && c <= 'z')
    {
      int js_idx = c - 'a';
      if (js_idx >= 0 && js_idx < (int)js_vec.size())
      {
        ROS_INFO("sent waypoint %c: [%s]",
                 'a' + js_idx, js_name_vec[js_idx].c_str());
        pub.publish(js_vec[js_idx]);
      }
      else
        ROS_INFO("only a-%c are defined in this file",
                 'a'+(int)js_vec.size()-1);
    }
    /*
    tf::transformTFToMsg(tf::Transform(target_quat, target_vec), ik_msg.t);
    ik_req_pub.publish(ik_msg);
    */
  }
  tcsetattr(0, TCSADRAIN, &cooked);
  printf("bai\n");
  fflush(0);
  return 0;
}

