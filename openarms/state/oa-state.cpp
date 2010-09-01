#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "openarms/ArmSensors.h"

static double g_joint_pos[8];
ros::Publisher *g_joint_pub = NULL;
static double g_encoder_offsets[8], g_encoder_scales[8];

void sensors_cb(const openarms::ArmSensors::ConstPtr &sensors)
{
  for (int i = 0; i < 4; i++)
  {
    int enc = sensors->encoder[i];
    if (enc > 5000)
      enc -= 10000;
    g_joint_pos[i] = enc * g_encoder_scales[i] + g_encoder_offsets[i];
  }
  for (int i = 4; i < 8; i++)
  {
    int enc = sensors->pos[i]; // should rename this sometime; these r the pots
    g_joint_pos[i] = enc * g_encoder_scales[i] + g_encoder_offsets[i];
  }

  // handle the gripper now
  //g_joint_pos[7] = (sensors->pos[7] - 216) / (371.0 - 216.0);

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
  {
    g_joint_pos[i] = 0;
    g_encoder_offsets[i] = 0;
    g_encoder_scales[i] = 1;
  }

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
      for (int i = 0; token && i < 8; i++, token = strtok(NULL, " \n"))
        g_encoder_offsets[i] = atof(token);
    else if (!strcmp(key, "encoder_scales"))
      for (int i = 0; token && i < 8; i++, token = strtok(NULL, " \n"))
        g_encoder_scales[i] = atof(token);
    else
      printf("unrecognized key: [%s]\n", token);
  }
  fclose(f);
  printf("enc offsets: ");
  for (int i = 0; i < 8; i++)
    printf(" %6.2f", g_encoder_offsets[i]);
  printf("\nenc scales: ");
  for (int i = 0; i < 8; i++)
    printf(" %6.4g", g_encoder_scales[i]);
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
