#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "openarms/ArmSensors.h"

static double g_joint_pos[7];
ros::Publisher *g_joint_pub = NULL;
static int32_t g_stepper_offsets[4], g_servo_offsets[3];
bool g_offset_init_complete = false;

void sensors_cb(const openarms::ArmSensors::ConstPtr &sensors)
{
  if (!g_offset_init_complete)
  {
    g_offset_init_complete = true;
    for (int i = 0; i < 4; i++)
      g_stepper_offsets[i] = sensors->pos[i];
  }

  // 10-microstep, 1.8 deg/step
  g_joint_pos[0] = (sensors->pos[0] - g_stepper_offsets[0]) / 2.0 / 10.0 * 1.8 * 3.1415 / 180.0 / 6.5; 
  g_joint_pos[1] = (sensors->pos[1] - g_stepper_offsets[1]) / 2.0 / 10.0 * 1.8 * 3.1415 / 180.0 / 10.133 + g_joint_pos[0] / 1.97; 
  g_joint_pos[2] = (sensors->pos[2] - g_stepper_offsets[2]) / 2.0 / 10.0 * 1.8 * 3.1415 / 180.0 / 10.133; 
  g_joint_pos[3] = (sensors->pos[3] - g_stepper_offsets[3]) / 2.0 / 10.0 * 1.8 * 3.1415 / 180.0 / 10.133;
  /*
  static int print_count = 0;
  if (print_count++ % 50 == 0)
    ROS_INFO("%10d %10f", sensors->pos[0], g_joint_pos[0]);
  */
  if (g_joint_pub)
  {
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(7);
    joint_state.position.resize(7);
    joint_state.name[0] = "shoulder1";
    joint_state.name[1] = "shoulder2";
    joint_state.name[2] = "shoulder3";
    joint_state.name[3] = "elbow";
    joint_state.name[4] = "wrist1";
    joint_state.name[5] = "wrist2";
    joint_state.name[6] = "wrist3";
    for (int i = 0; i < 7; i++)
      joint_state.position[i] = g_joint_pos[i];
    joint_state.header.stamp = ros::Time::now();
    g_joint_pub->publish(joint_state);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oa_state");
  ros::NodeHandle n;
  for (uint32_t i = 0; i < 7; i++)
    g_joint_pos[i] = 0;
  for (uint32_t i = 0; i < 4; i++)
    g_stepper_offsets[i] = 0;
  for (uint32_t i = 0; i < 3; i++)
    g_servo_offsets[i] = 0;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  g_joint_pub = &joint_pub; // ugly ugly
  ros::Subscriber sensor_sub = n.subscribe("arm_sensors", 1, sensors_cb);
  tf::TransformBroadcaster tf_broadcaster;
  ros::Rate loop_rate(100);
  geometry_msgs::TransformStamped world_trans;
  world_trans.header.frame_id = "world";
  world_trans.child_frame_id = "torso_link";
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
