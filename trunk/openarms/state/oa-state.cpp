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

  joint_state.name.resize(1);
  joint_state.position.resize(1);
  joint_state.name[0] = "shoulderlift_joint";
  joint_state.position[0] = 0;
  double lift_angle = 0;
  while (ros::ok())
  {
    lift_angle += 0.01;
    joint_state.position[0] = lift_angle;
    joint_state.header.stamp = ros::Time::now();
    joint_pub.publish(joint_state);
    world_trans.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(world_trans);

    loop_rate.sleep();
  }
  return 0;
}
