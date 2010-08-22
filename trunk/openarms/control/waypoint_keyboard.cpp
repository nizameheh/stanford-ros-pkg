// copyright 2010 morgan quigley bsd license blah blah
// lots of code lifted from the teleop_head_keyboard program by Kevin Watts
#include <cstdlib>
#include <cstdio>
#include <termios.h>

struct termios cooked, raw; // mmm makes me hungry

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_keyboard"); // remember he-man?
  ros::NodeHandle n;
  ros::Publisher ik_req_pub = n.advertise<openarms::ArmIKRequest>("ik_request", 1);
  char c;
  tcgetattr(0, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VTIME] = 1;
  raw.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &raw);
  puts("greetings. ctrl-c to exit.");
  //tf::Transform t(btQuaternion::getIdentity(), btVector3(0, 0, 0));
  btVector3 target_vec(-0.2, 0.45, -0.3);
  btQuaternion target_quat(btQuaternion(btVector3(1, 0, 0), 3.14));
  openarms::ArmIKRequest ik_msg;

  const double DELTA = 0.01, QUAT_DELTA = 0.05;
  
  while (ros::ok())
  {
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
      printf("pressed: %c\n", c);
    }
    if (c == 0x2a)
      break;
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

