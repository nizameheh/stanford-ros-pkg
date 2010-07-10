#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "openarms/ArmSensors.h"

static double g_joint_pos[8];
ros::Publisher *g_joint_pub = NULL;
static int32_t g_stepper_offsets[4], g_servo_offsets[3];
bool g_offset_init_complete = false;
int32_t g_servo_wraps[3];
enum servo_wrap_t { COMING_FROM_LOW, COMING_FROM_HIGH, NOT_IN_WRAP} g_servo_in_wrap[3];

void sensors_cb(const openarms::ArmSensors::ConstPtr &sensors)
{
  if (!g_offset_init_complete)
  {
    g_offset_init_complete = true;
    for (int i = 0; i < 4; i++)
      g_stepper_offsets[i] = sensors->pos[i];
    for (int i = 0; i < 3; i++)
      g_servo_offsets[i] = sensors->pos[i+4];
  }

  // 10-microstep, 1.8 deg/step
  g_joint_pos[0] = (sensors->pos[0] - g_stepper_offsets[0]) / 2.0 / 10.0 * 1.8 * 3.1415 / 180.0 / 6.5; 
  g_joint_pos[1] = (sensors->pos[1] - g_stepper_offsets[1]) / 2.0 / 10.0 * 1.8 * 3.1415 / 180.0 / 10.133 + g_joint_pos[0] / 1.97; 
  g_joint_pos[2] = (sensors->pos[2] - g_stepper_offsets[2]) / 2.0 / 10.0 * 1.8 * 3.1415 / 180.0 / 14;
  g_joint_pos[3] = (sensors->pos[3] - g_stepper_offsets[3]) / 2.0 / 10.0 * 1.8 * 3.1415 / 180.0 / 14 - 1.57; // assume we init with elbow straight down

  const int32_t WRAP_THRESH = 30, NOT_IN_WRAP_THRESH = 60;
  for (int i = 0; i < 3; i++)
  {
    int32_t pos = sensors->pos[i+4];
    if (pos < WRAP_THRESH)
    {
      if (g_servo_in_wrap[i] == COMING_FROM_HIGH)
        g_servo_wraps[i]++;
      g_servo_in_wrap[i] = COMING_FROM_LOW;
    }
    else if (pos > 1024 - WRAP_THRESH)
    {
      if (g_servo_in_wrap[i] == COMING_FROM_LOW)
        g_servo_wraps[i]--;
      g_servo_in_wrap[i] = COMING_FROM_HIGH;
    }
    else if ((pos >= WRAP_THRESH && pos < NOT_IN_WRAP_THRESH) ||
             (pos > 1024 - NOT_IN_WRAP_THRESH && pos <= 1024 - WRAP_THRESH))
    {
      g_servo_in_wrap[i] = NOT_IN_WRAP;
    }
    if (g_servo_in_wrap[i] != NOT_IN_WRAP &&
        (pos > NOT_IN_WRAP_THRESH && pos < 1024 - NOT_IN_WRAP_THRESH))
    {
      if (g_servo_in_wrap[i] == COMING_FROM_HIGH)
        pos = 1023;
      else
        pos = 0;
    }
    g_joint_pos[i+4] = (pos - g_servo_offsets[i] + 1024 * g_servo_wraps[i]) * 2 * 3.1415 / 1023.0 / 2.8;
    /*
    if (i == 1)
      printf("%8d %4d %4d %4d %.3f\n", 
             pos, sensors->pos[i+4],
             g_servo_wraps[i], g_servo_in_wrap[i],
             g_joint_pos[i+4]);
    */
  }
  // handle the gripper now
  g_joint_pos[7] = (sensors->pos[7] - 216) / (371.0 - 216.0);
  // assume servos are all geared down 3:1
  //g_joint_pos[4] = (sensors->pos[4] - g_servo_offsets[0])*2*3.14/1024.0/4.0;
  //g_joint_pos[5] = (sensors->pos[5] - g_servo_offsets[1])*2*3.14/1024.0/3.0;
  //g_joint_pos[6] = (sensors->pos[6] - g_servo_offsets[2])*2*3.14/1024.0/3.0;

  /*
  static int print_count = 0;
  if (print_count++ % 50 == 0)
    ROS_INFO("%10d %10f", sensors->pos[0], g_joint_pos[0]);
  */
  if (g_joint_pub)
  {
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(8);
    joint_state.position.resize(8);
    joint_state.name[0] = "shoulder1";
    joint_state.name[1] = "shoulder2";
    joint_state.name[2] = "shoulder3";
    joint_state.name[3] = "elbow";
    joint_state.name[4] = "wrist1";
    joint_state.name[5] = "wrist2";
    joint_state.name[6] = "wrist3";
    joint_state.name[7] = "gripper";
    for (int i = 0; i < 8; i++)
      joint_state.position[i] = g_joint_pos[i];
    joint_state.header.stamp = ros::Time::now();
    g_joint_pub->publish(joint_state);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oa_state");
  ros::NodeHandle n;
  for (uint32_t i = 0; i < 8; i++)
    g_joint_pos[i] = 0;
  for (uint32_t i = 0; i < 4; i++)
    g_stepper_offsets[i] = 0;
  for (uint32_t i = 0; i < 3; i++)
  {
    g_servo_offsets[i] = 0;
    g_servo_wraps[i] = 0;
    g_servo_in_wrap[i] = NOT_IN_WRAP;
  }

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
