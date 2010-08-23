#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "openarms/ArmSensors.h"

static double g_joint_pos[8];
ros::Publisher *g_joint_pub = NULL;
static double g_encoder_offsets[7], g_gear_ratios[3], g_encoder_signs[7];
int32_t g_servo_wraps[3];
enum servo_wrap_t { COMING_FROM_LOW, COMING_FROM_HIGH, NOT_IN_WRAP} g_servo_in_wrap[3];

void sensors_cb(const openarms::ArmSensors::ConstPtr &sensors)
{
  for (int i = 0; i < 4; i++)
  {
    int enc = sensors->encoder[i];
    if (enc > 5000)
      enc -= 10000;
    g_joint_pos[i] = enc / 10000.0 * 2 * M_PI * g_encoder_signs[i] + 
                     g_encoder_offsets[i];
  }


  const int32_t DEADBAND_WIDTH = 8;
  const int32_t WRAP_THRESH_LOW = DEADBAND_WIDTH;
  const int32_t WRAP_THRESH_HIGH = 1023 - DEADBAND_WIDTH;
  for (int i = 0; i < 3; i++)
  {
    int32_t pos = sensors->pos[i+4];
    if (pos < WRAP_THRESH_LOW)
    {
      if (g_servo_in_wrap[i] == COMING_FROM_HIGH)
        g_servo_wraps[i]++;
      g_servo_in_wrap[i] = COMING_FROM_LOW;
    }
    else if (pos > WRAP_THRESH_HIGH)
    {
      if (g_servo_in_wrap[i] == COMING_FROM_LOW)
        g_servo_wraps[i]--;
      g_servo_in_wrap[i] = COMING_FROM_HIGH;
    }
    else if  (pos < WRAP_THRESH_LOW + DEADBAND_WIDTH ||
              pos > WRAP_THRESH_HIGH - DEADBAND_WIDTH)
    {
      g_servo_in_wrap[i] = NOT_IN_WRAP;
    }
    else if (g_servo_in_wrap[i] != NOT_IN_WRAP)
    {
      // we're in the dead band scratch zone. 
      if (g_servo_in_wrap[i] == COMING_FROM_HIGH)
        pos = 1023;
      else
        pos = 0;
    }
    g_joint_pos[i+4] = g_encoder_offsets[i+4] +
                       (pos + 1024 * g_servo_wraps[i]) * g_encoder_signs[i+4] *
                        2 * 3.1415 / 1023.0 / g_gear_ratios[i];
    /*
    if (i == 1)
      printf("%8d %4d %4d %4d %.3f\n", 
             pos, sensors->pos[i+4],
             g_servo_wraps[i], g_servo_in_wrap[i],
             g_joint_pos[i+4]);
    */
  }
  /*
  printf("%+5d %+5d %+5d\n",
         g_servo_wraps[0], g_servo_wraps[1], g_servo_wraps[2]);
  */
  
  // enforce joint limits on the wrap count
  // wrist pitch. enforce joint limits on wrap count

  if (g_joint_pos[5] < -2.5)
  {
    ROS_ERROR("woah, wrist overwrap negative");
    g_servo_wraps[1]++;
  }
  else if (g_joint_pos[5] > 2.5)
  {
    ROS_ERROR("woah, wrist overwrap positive");
    g_servo_wraps[1]--;
  }

  // enforce joint limits on forearm roll
  /*
  if (g_joint_pos[4] < -3.0)
    g_servo_wraps[0]++;
  else if (g_joint_pos[4] > 3.1)
    g_servo_wraps[0]--;
  */

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
  if (argc != 2)
  {
    ROS_INFO("usage: oa_state CALIBRATION");
    return 1;
  }
  
  for (uint32_t i = 0; i < 8; i++)
    g_joint_pos[i] = 0;
  for (uint32_t i = 0; i < 7; i++)
  {
    g_encoder_offsets[i] = 0;
    g_encoder_signs[i] = 1;
  }
  for (uint32_t i = 0; i < 3; i++)
  {
    g_gear_ratios[i] = 0;
    g_servo_wraps[i] = 0;
    g_servo_in_wrap[i] = NOT_IN_WRAP;
  }
  g_servo_wraps[0] = -1;
  g_servo_wraps[1] = -1;
  g_servo_wraps[2] = -1;

  FILE *f = fopen(argv[1], "r");
  if (!f)
  {
    ROS_ERROR("woah there. couldn't open %s", argv[1]);
    return 1;
  }
  // read in the calibration data...
  while (!feof(f))
  {
    char line[1000];
    if (!fgets(line, sizeof(line), f))
      break;
    if (!strlen(line))
      continue;
    char *key = strtok(line, " \n");
    char *token = strtok(NULL, " \n");
    if (!strcmp(key, "encoder_offsets"))
      for (int i = 0; token && i < 7; i++, token = strtok(NULL, " \n"))
        g_encoder_offsets[i] = atof(token);
    else if (!strcmp(key, "gear_ratios"))
      for (int i = 0; token && i < 3; i++, token = strtok(NULL, " \n"))
        g_gear_ratios[i] = atof(token);
    else if (!strcmp(key, "encoder_signs"))
      for (int i = 0; token && i < 7; i++, token = strtok(NULL, " \n"))
        g_encoder_signs[i] = atof(token);
    else
      printf("unrecognized key: [%s]\n", token);
  }
  fclose(f);
  printf("enc offsets: ");
  for (int i = 0; i < 7; i++)
    printf(" %8.2f", g_encoder_offsets[i]);
  printf("\ngear ratios: ");
  for (int i = 0; i < 3; i++)
    printf(" %8.2f", g_gear_ratios[i]);
  printf("\n");
  
  ros::NodeHandle n;

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  g_joint_pub = &joint_pub; // ugly ugly
  ros::Subscriber sensor_sub = n.subscribe("arm_sensors", 1, sensors_cb);
  tf::TransformBroadcaster tf_broadcaster;
  ros::Rate loop_rate(30); // just fast enough to be new enough so tf is happy
  geometry_msgs::TransformStamped world_trans;
  world_trans.header.frame_id = "world";
  world_trans.child_frame_id = "torso_link";
  world_trans.transform.translation.x = 0;
  world_trans.transform.translation.y = 0;
  world_trans.transform.translation.z = 0;
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
