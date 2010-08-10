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
#include "tf/transform_listener.h"
#include "openarms/ArmIKRequest.h"
#include "std_msgs/UInt8.h"
#include "visualization_msgs/Marker.h"
using std::vector;

ros::Publisher *g_target_pub = NULL, *g_marker_pub = NULL;
tf::TransformBroadcaster *g_tf_broadcaster = NULL;
tf::TransformListener *g_tf_listener = NULL;
geometry_msgs::Quaternion q0, q1, q2, q3;
bool init_complete[4];
ros::Publisher *g_joint_pub = NULL;
tf::Transform g_workspace_center;
int g_posture_bangbang = 0;
void q0_cb(const geometry_msgs::Quaternion::ConstPtr &q)
{
  q0 = *q;
  init_complete[0] = true;
  if (!init_complete[1] || !init_complete[2] || !init_complete[3])
    return; // be patient

  btTransform t0(btQuaternion(q0.x, q0.y, q0.z, q0.w));
  btTransform t1(btQuaternion(q1.x, q1.y, q1.z, q1.w));
  btTransform t2(btQuaternion(q2.x, q2.y, q2.z, q2.w));
  btTransform t3(btQuaternion(q3.x, q3.y, q3.z, q3.w));

  //t1.setOrigin(t1 * btVector3(0, 0.1, 0));
  //t1.setRotation(t1.getRotation().inverse());
  btTransform upperarm_shirt_offset(btQuaternion::getIdentity(),
                                    btVector3(-0.1, 0, 0));
  //t1 = t1 * upperarm_shirt_offset;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "master";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.8;
  marker.color.b = 0.3;
  g_marker_pub->publish(marker);

#if 0
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
  js.name[0] = "human_shoulder_flexion";
  js.name[1] = "human_shoulder_abduction";
  js.name[2] = "human_humeral_rotation";
  js.name[3] = "human_elbow";
  js.name[4] = "human_forearm_roll";
  js.name[5] = "human_wrist_pitch";
  js.name[6] = "human_wrist_roll";
  g_joint_pub->publish(js);
#endif 
  tf::StampedTransform t0s(t0, ros::Time::now(), "world", "sp0");
  tf::StampedTransform t1s(t1, ros::Time::now(), "world", "sp1");
  tf::StampedTransform t2s(t2, ros::Time::now(), "world", "sp2");
  tf::StampedTransform t3s(t3, ros::Time::now(), "world", "sp3");

  geometry_msgs::TransformStamped t0_msg, t1_msg, t2_msg, t3_msg;

  tf::transformStampedTFToMsg(t0s, t0_msg);
  tf::transformStampedTFToMsg(t1s, t1_msg);
  tf::transformStampedTFToMsg(t2s, t2_msg);
  tf::transformStampedTFToMsg(t3s, t3_msg);

  g_tf_broadcaster->sendTransform(t0_msg);
  g_tf_broadcaster->sendTransform(t1_msg);
  g_tf_broadcaster->sendTransform(t2_msg);
  g_tf_broadcaster->sendTransform(t3_msg);
#if 0
  geometry_msgs::TransformStamped world_trans;
  world_trans.header.frame_id = "world";
  world_trans.child_frame_id = "human_torso_link";
  world_trans.transform.translation.x = 0;
  world_trans.transform.translation.y = 0;
  world_trans.transform.translation.z = 0;
  world_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  world_trans.header.stamp = ros::Time::now();
  g_tf_broadcaster->sendTransform(world_trans);

  // broadcast the workspace center
  geometry_msgs::TransformStamped workspace_center_trans;
  workspace_center_trans.header.frame_id = "world";
  workspace_center_trans.header.stamp = ros::Time::now();
  workspace_center_trans.child_frame_id = "workspace_center";
  btVector3 workspace_center(0.35, 0.4, -0.3);
  workspace_center_trans.transform.translation.x = workspace_center.x();
  workspace_center_trans.transform.translation.y = workspace_center.y();
  workspace_center_trans.transform.translation.z = workspace_center.z();
  workspace_center_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  g_tf_broadcaster->sendTransform(workspace_center_trans);

  tf::StampedTransform tool_in_workspace;
  try
  {
    /*
    g_tf_listener->lookupTransform("workspace_center", "human_tool_link",
                                   ros::Time(0), tool_in_workspace);
    */
    g_tf_listener->lookupTransform("world", "human_tool_link",
                                   ros::Time(0), tool_in_workspace);
    geometry_msgs::Transform target_msg;
    tf::transformTFToMsg(tool_in_workspace, target_msg);
    openarms::ArmIKRequest ik_req_msg;
    ik_req_msg.t = target_msg;
    if (g_posture_bangbang == 0)
    {
      ik_req_msg.posture = 0.5;
      ik_req_msg.posture_gain = 0.0;
    }
    else if (g_posture_bangbang == 1)
    {
      ik_req_msg.posture = 0.05;
      ik_req_msg.posture_gain = 0.5;
    }
    else if (g_posture_bangbang == 2)
    {
      ik_req_msg.posture = 0.95;
      ik_req_msg.posture_gain = 0.5;
    }
    g_target_pub->publish(ik_req_msg);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
#endif
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

void posture_bangbang_cb(const std_msgs::UInt8::ConstPtr &msg)
{
  g_posture_bangbang = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "master_spacepointshirt"); // remember he-man? awesome.
  ros::NodeHandle n;
  ros::Subscriber q0_sub = n.subscribe("spacepoint1_quat", 1, q0_cb);
  ros::Subscriber q1_sub = n.subscribe("spacepoint2_quat", 1, q1_cb);
  ros::Subscriber q2_sub = n.subscribe("spacepoint3_quat", 1, q2_cb);
  ros::Subscriber q3_sub = n.subscribe("spacepoint4_quat", 1, q3_cb);
  ros::Subscriber posture_sub = n.subscribe("posture_bangbang", 1, posture_bangbang_cb);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("master_state", 1);
  ros::Publisher target_pub = n.advertise<openarms::ArmIKRequest>("ik_request", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  g_target_pub = &target_pub;
  g_joint_pub = &joint_pub;
  g_marker_pub = &marker_pub;
  for (int i = 0; i < 4; i++)
    init_complete[i] = false;
  g_workspace_center = btTransform(btQuaternion::getIdentity(),
                                   btVector3(0, 0, 0.3));

  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformListener tf_listener;
  g_tf_broadcaster = &tf_broadcaster;
  g_tf_listener = &tf_listener;
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

