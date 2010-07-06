// copyright 2010 morgan quigley bsd license blah blah
// lots of code lifted from the teleop_head_keyboard program by Kevin Watts
#include <cstdlib>
#include <cstdio>
#include <termios.h>

#include <ros/ros.h>

struct termios cooked, raw; // mmm makes me hungry


int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_keyboard"); // remember he-man?
  ros::NodeHandle n;
  char c;
  tcgetattr(0, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VTIME] = 1;
  raw.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &raw);
  puts("greetings. ctrl-c to exit.");
  while (ros::ok())
  {
    ssize_t nread = read(0, &c, 1);
    if (nread < 0)
    {
      perror("read():");
      exit(-1);
    }
    else if (nread == 0)
    {
      ros::spinOnce();
      continue;
    }
    bool updated = true; // assume the best
    switch (c)
    {
      case 'a':
        printf("a\n");
        break;
      case 'x':
        printf("x\n");
        break;
      case 'w':
        printf("w\n");
        break;
      case 'd':
        printf("d\n");
        break;
      default:
        updated = false; // plan for the worst
        break;
    }
    if (updated)
    {
      printf("sending delta pose\n");
    }
  }
  tcsetattr(0, TCSADRAIN, &cooked);
  printf("bai\n");
  fflush(0);
  return 0;
}

