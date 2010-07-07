// copyright 2010 morgan quigley bsd license blah blah
// lots of code lifted from the teleop_head_keyboard program by Kevin Watts
#include <cstdlib>
#include <cstdio>

#include <ros/ros.h>
#include "geometry_msgs/Transform.h"
#include "LinearMath/btTransform.h"
#include "tf/transform_datatypes.h"
#include "joy/Joy.h"

ros::Publisher *g_target_pub = NULL;
double g_joy_axes[4];

void joy_cb(const joy::Joy::ConstPtr &joy_msg)
{
  if (joy_msg->axes.size() >= 4)
  {
    for (int i = 0; i < 4; i++)
      g_joy_axes[i] = joy_msg->axes[i];
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_gamepad"); // remember he-man? awesome.
  ros::NodeHandle n;
  ros::Publisher tf_pub = n.advertise<geometry_msgs::Transform>("target_frame", 1);
  ros::Subscriber joy_sub = n.subscribe("joy", 1, joy_cb);
  g_target_pub = &tf_pub;
  for (int i = 0; i < 4; i++)
    g_joy_axes[i] = 0;
  puts("greetings. ctrl-c to exit.");
  //t = tf::Transform(btQuaternion::getIdentity(), btVector3(0,0,0));
  
  ros::Rate loop_rate(100);

  geometry_msgs::Transform tf_msg;
  //tf::Transform target_tf;
  btVector3 target_vec(0,0,0);

  while (ros::ok())
  {
    target_vec += 0.001 * btVector3(g_joy_axes[0],
                                    g_joy_axes[1],
                                    g_joy_axes[2]);
    tf::transformTFToMsg(tf::Transform(btQuaternion::getIdentity(), target_vec),
                         tf_msg);
    g_target_pub->publish(tf_msg);
    loop_rate.sleep();
    ros::spinOnce();
  }
  printf("bai\n");
  return 0;
}

