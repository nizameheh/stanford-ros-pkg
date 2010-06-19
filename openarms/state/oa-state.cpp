#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "openarms/ArmSensors.h"

const uint32_t NUM_JOINTS = 2;
static double g_joint_bias[2];
static double g_joint_pos[2];
ros::Publisher *g_joint_pub = NULL;

void sensors_cb(const openarms::ArmSensors::ConstPtr &sensors)
{
  // 10-microstep, 1.8 deg/step
  g_joint_pos[0] = g_joint_bias[0] - sensors->pos[0] / 2.0 / 10.0 * 1.8 * 3.1415 / 180.0 / 6.0; 
  static int print_count = 0;
  if (print_count++ % 50 == 0)
    ROS_INFO("%10d %10f", sensors->pos[0], g_joint_pos[0]);
  if (g_joint_pub)
  {
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.name[0] = "shoulderpitch_joint";
    joint_state.position[0] = g_joint_pos[0];
    joint_state.name[1] = "shoulderlift_joint";
    joint_state.position[1] = 0;
    joint_state.header.stamp = ros::Time::now();
    g_joint_pub->publish(joint_state);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oa_state");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");
  for (uint32_t i = 0; i < NUM_JOINTS; i++)
    g_joint_bias[i] = 0;
  n_private.getParam("joint0bias", g_joint_bias[0]);
  n_private.getParam("joint1bias", g_joint_bias[1]);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  g_joint_pub = &joint_pub; // ugly ugly
  ros::Subscriber sensor_sub = n.subscribe("arm_sensors", 1, sensors_cb);
  tf::TransformBroadcaster tf_broadcaster;
  ros::Rate loop_rate(100);
  geometry_msgs::TransformStamped world_trans;
  world_trans.header.frame_id = "world";
  world_trans.child_frame_id = "torso";
  world_trans.transform.translation.x = 0;
  world_trans.transform.translation.y = 0;
  world_trans.transform.translation.z = 0;
  //world_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(90*RAD2DEG,0,0);
  world_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

  while (ros::ok())
  {
    world_trans.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(world_trans);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
