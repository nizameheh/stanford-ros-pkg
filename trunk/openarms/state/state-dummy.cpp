#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oa_state");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
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
  joint_state.position[0] = 0;
  joint_state.position[1] = 0;
  joint_state.position[2] = 0;
  joint_state.position[3] = 0;
  joint_state.position[4] = 0;
  joint_state.position[5] = 0;
  joint_state.position[6] = 0;

  while (ros::ok())
  {
    world_trans.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(world_trans);
    joint_state.header.stamp = ros::Time::now();
    joint_pub.publish(joint_state);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

