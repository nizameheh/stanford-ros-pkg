// copyright 2010 morgan quigley bsd license blah blah
// lots of code lifted from the teleop_head_keyboard program by Kevin Watts
#include <cstdlib>
#include <cstdio>

#include <ros/ros.h>
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include "LinearMath/btTransform.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
using std::vector;

//ros::Publisher *g_target_pub = NULL;
tf::TransformBroadcaster *g_tf_broadcaster = NULL;
geometry_msgs::Quaternion q0, q1, q2, q3;
bool init_complete[4];
ros::Publisher *g_joint_pub = NULL;

void q0_cb(const geometry_msgs::Quaternion::ConstPtr &q)
{
  q0 = *q;
  init_complete[0] = true;
  if (!init_complete[1] || !init_complete[2] || !init_complete[3])
    return; // be patient

  //btVector3 target_vec(0,0,0);

  btTransform t0(btQuaternion(q0.x, q0.y, q0.z, q0.w));
  btTransform t1(btQuaternion(q1.x, q1.y, q1.z, q1.w));
  btTransform t2(btQuaternion(q2.x, q2.y, q2.z, q2.w));
  btTransform t3(btQuaternion(q3.x, q3.y, q3.z, q3.w));

  t0.setRotation(t0.getRotation() * btQuaternion(3.14, 0, 0));

  btQuaternion t01(t0.getRotation().inverse() * t1.getRotation());
  btQuaternion t12(t1.getRotation().inverse() * t2.getRotation());
  btQuaternion t23(t2.getRotation().inverse() * t3.getRotation());
  double roll, pitch, yaw;
  btMatrix3x3(t01).getRPY(roll, pitch, yaw);

  vector<double> joint_pos;
  joint_pos.resize(7);
  for (int i = 0; i < 7; i++)
    joint_pos[i] = 0;
  joint_pos[0] = pitch;
  joint_pos[1] = -yaw;

  btMatrix3x3(t12).getRPY(roll, pitch, yaw);
  joint_pos[2] = 2*pitch - M_PI/4;
  joint_pos[3] = 1.57 - yaw + 3.14;
  joint_pos[4] = roll + 3.14;

  btMatrix3x3(t23).getRPY(roll, pitch, yaw);
  joint_pos[5] = pitch;
  joint_pos[6] = roll;

  //printf("%.3f %.3f %.3f\n", roll, pitch, yaw);
/*
  printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
         joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3],
         joint_pos[4], joint_pos[5], joint_pos[6]);
*/

  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();
  js.position = joint_pos;
  js.name.resize(7);
  js.name[0] = "shoulder_flexion";
  js.name[1] = "shoulder_abduction";
  js.name[2] = "humeral_rotation";
  js.name[3] = "elbow";
  js.name[4] = "forearm_roll";
  js.name[5] = "wrist_pitch";
  js.name[6] = "wrist_roll";
  g_joint_pub->publish(js);
  
  tf::StampedTransform t0s(t0, ros::Time::now(), "world", "sp0");
  tf::StampedTransform t1s(t1, ros::Time::now(), "world", "sp1");
  tf::StampedTransform t2s(t2, ros::Time::now(), "world", "sp2");

  geometry_msgs::TransformStamped t0_msg, t1_msg, t2_msg;

  tf::transformStampedTFToMsg(t0s, t0_msg);
  tf::transformStampedTFToMsg(t1s, t1_msg);
  tf::transformStampedTFToMsg(t2s, t2_msg);

  g_tf_broadcaster->sendTransform(t0_msg);
  g_tf_broadcaster->sendTransform(t1_msg);
  g_tf_broadcaster->sendTransform(t2_msg);

  geometry_msgs::TransformStamped world_trans;
  world_trans.header.frame_id = "world";
  world_trans.child_frame_id = "torso_link";
  world_trans.transform.translation.x = 0;
  world_trans.transform.translation.y = 0;
  world_trans.transform.translation.z = 0;
  world_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  world_trans.header.stamp = ros::Time::now();
  g_tf_broadcaster->sendTransform(world_trans);
}

void q1_cb(const geometry_msgs::Quaternion::ConstPtr &q)
{
  q1 = *q;
  init_complete[1] = true;
}

void q2_cb(const geometry_msgs::Quaternion::ConstPtr &q)
{
  q2 = *q;
  init_complete[2] = true;
}

void q3_cb(const geometry_msgs::Quaternion::ConstPtr &q)
{
  q3 = *q;
  init_complete[3] = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_spacepointshirt"); // remember he-man? awesome.
  ros::NodeHandle n;
  ros::Subscriber q0_sub = n.subscribe("spacepoint1_quat", 1, q0_cb);
  ros::Subscriber q1_sub = n.subscribe("spacepoint2_quat", 1, q1_cb);
  ros::Subscriber q2_sub = n.subscribe("spacepoint3_quat", 1, q2_cb);
  ros::Subscriber q3_sub = n.subscribe("spacepoint4_quat", 1, q3_cb);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("master_state", 1);
  g_joint_pub = &joint_pub;
  for (int i = 0; i < 4; i++)
    init_complete[i] = false;

  tf::TransformBroadcaster tf_broadcaster;
  g_tf_broadcaster = &tf_broadcaster;
  puts("greetings. ctrl-c to exit.");
  //t = tf::Transform(btQuaternion::getIdentity(), btVector3(0,0,0));
  
  ros::Rate loop_rate(100);


  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }
  printf("bai\n");
  return 0;
}

