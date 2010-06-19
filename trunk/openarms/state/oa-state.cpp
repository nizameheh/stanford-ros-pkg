#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "openarms/ArmSensors.h"

const uint32_t NUM_JOINTS = 2;
static double g_joint_bias[2];
static double g_joint_pos[2];

void sensors_cb(const openarms::ArmSensors::ConstPtr &sensors)
{
  // 10-microstep, 1.8 deg/step
  g_joint_pos[0] = g_joint_bias[0] - sensors->pos[0] / 10.0 * 1.8 * 3.1415 / 180.0 / 8.0; 
  ROS_INFO("%10d %10f\n", sensors->pos[0], g_joint_pos[0]);
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
  ros::Subscriber sensor_sub = n.subscribe("arm_sensors", 1, sensors_cb);
  tf::TransformBroadcaster tf_broadcaster;
  ros::Rate loop_rate(30);
  const double RAD2DEG = M_PI/180;
  sensor_msgs::JointState joint_state;
  geometry_msgs::TransformStamped world_trans;
  world_trans.header.frame_id = "world";
  world_trans.child_frame_id = "torso";
  world_trans.transform.translation.x = 0;
  world_trans.transform.translation.y = 0;
  world_trans.transform.translation.z = 0;
  //world_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(90*RAD2DEG,0,0);
  world_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0] = "shoulderpitch_joint";
  joint_state.position[0] = 0;
  joint_state.name[1] = "shoulderlift_joint";
  joint_state.position[1] = 0;
  double lift_angle = 0;
  while (ros::ok())
  {
    //lift_angle += 0.01;
    joint_state.position[0] = lift_angle;
    joint_state.header.stamp = ros::Time::now();
    joint_pub.publish(joint_state);
    world_trans.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(world_trans);

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
