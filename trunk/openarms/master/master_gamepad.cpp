// copyright 2010 morgan quigley bsd license blah blah
// lots of code lifted from the teleop_head_keyboard program by Kevin Watts
#include <cstdlib>
#include <cstdio>

#include <ros/ros.h>
#include "geometry_msgs/Transform.h"
#include "LinearMath/btTransform.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "joy/Joy.h"

const static int32_t MAX_BUTTONS = 10;
ros::Publisher *g_target_pub = NULL;
static double g_joy_axes[4];
static int    g_joy_buttons[MAX_BUTTONS];

void joy_cb(const joy::Joy::ConstPtr &joy_msg)
{
  if (joy_msg->axes.size() >= 4)
    for (int i = 0; i < 4; i++)
      g_joy_axes[i] = joy_msg->axes[i];
  for (int i = 0; i < MAX_BUTTONS && i < (int)joy_msg->buttons.size(); i++)
    g_joy_buttons[i] = joy_msg->buttons[i];
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
  for (int i = 0; i < MAX_BUTTONS; i++)
    g_joy_buttons[i] = 0;
  puts("greetings. ctrl-c to exit.");

  //t = tf::Transform(btQuaternion::getIdentity(), btVector3(0,0,0));
  
  ros::Rate loop_rate(50);

  geometry_msgs::Transform tf_msg;
  btVector3 target_vec(0, 0.75, -0.2);

  tf::TransformBroadcaster tf_broadcaster;
  btQuaternion orient(btQuaternion(btVector3(1, 0, 0), -2.5)); // *
                      //btQuaternion(btVector3(1, 0, 0), 2.50));

  while (ros::ok())
  {
    if (!g_joy_buttons[5])
    {
      double speed = (g_joy_buttons[4] ? 0.005 : 0.001);
      target_vec += speed * btVector3(-g_joy_axes[0],
                                       g_joy_axes[1],
                                       g_joy_axes[3]);
    }
    else
    {
      double speed = (g_joy_buttons[4] ? 0.040 : 0.01);
      orient = btQuaternion(btVector3(1, 0, 0), speed * g_joy_axes[1]) * orient;
      orient = btQuaternion(btVector3(0, 0, 1), speed * g_joy_axes[0]) * orient;
      orient = orient * btQuaternion(btVector3(0, 0, 1), speed * g_joy_axes[2]);
      orient.normalize();
    }
    /*
    tf::transformTFToMsg(tf::Transform(btQuaternion::getIdentity(), target_vec),
                         tf_msg);
    */
    tf::StampedTransform t_target_in_world(tf::Transform(orient, target_vec),
                                           ros::Time::now(),
                                           "world", "ik_target");
    geometry_msgs::TransformStamped t_target_msg;
    tf::transformStampedTFToMsg(t_target_in_world, t_target_msg);
    tf_broadcaster.sendTransform(t_target_msg);
    //g_target_pub->publish(tf_msg);
    loop_rate.sleep();
    ros::spinOnce();
  }
  printf("bai\n");
  return 0;
}

