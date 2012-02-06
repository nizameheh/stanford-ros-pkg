#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <hydra/Calib.h>
#include <tf/transform_listener.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>
#include <geometry_msgs/Twist.h>
#include <LinearMath/btVector3.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

static ros::Publisher *g_pose_pub[2] = { NULL, NULL };
static ros::Publisher *g_gripper_pub[2] = { NULL, NULL };
static ros::Publisher *g_base_pub = NULL;
static const int g_skip_rate = 5; // publish at 50 hz for now ?
static tf::TransformBroadcaster *g_tf_broadcaster = NULL;

static enum g_mode_t { MODE_IDLE, MODE_ACQUIRE, MODE_TRACK } g_mode = MODE_IDLE;
static ros::Time g_acquire_start;
static const double ACQUIRE_LENGTH = 4.0f;
static tf::TransformListener *g_listener = NULL;

double scale_and_clamp(const double x, const double scale, const double clamp)
{
  double y = x / scale * clamp; // nominally should be in [-clamp, clamp] ...
  if (y > clamp)
    y = clamp;
  else if (y < -clamp)
    y = -clamp;
  return y;
}

void print_mat(const char *msg, const btMatrix3x3 &m)
{
  if (msg)
    printf("%s\n", msg);
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
      printf("%+0.3f ", m[i][j]);
    printf("\n");
  }
}

void hydra_cb(const hydra::Calib &msg)
{
  if (!g_pose_pub[0] || !g_pose_pub[1] || !g_base_pub)
    return;
  static int s_skip_count = 0;
  if (++s_skip_count < g_skip_rate)
    return; // talk to the hand
  s_skip_count = 0;

  static btVector3 prev_sane_pos[2];
  btVector3 sane_pos[2];
  for (int s = 0; s < 2; s++)
    tf::vector3MsgToTF(msg.paddles[s].transform.translation, sane_pos[s]);

  for (int s = 0; s < 2; s++)
    if (sane_pos[s].x() > 0) // bogus, we've flipped to the other side.
      sane_pos[s] = prev_sane_pos[s];
    else
      prev_sane_pos[s] = sane_pos[s]; // we're ok

  sane_pos[0] += btVector3(1.0,  0, -0.2);
  sane_pos[1] += btVector3(1.0,  0, -0.2);

  geometry_msgs::TransformStamped rx_tf;
  rx_tf.header.stamp = ros::Time::now();
  rx_tf.header.frame_id = "world";
  rx_tf.child_frame_id = "hydra_left";
  rx_tf.transform = msg.paddles[0].transform;
  g_tf_broadcaster->sendTransform(rx_tf);
  rx_tf.child_frame_id = "hydra_right";
  rx_tf.transform = msg.paddles[1].transform;
  g_tf_broadcaster->sendTransform(rx_tf);

  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

  // this is horrible. fix sometime.
  if (g_mode == MODE_IDLE)
  {
    if (msg.paddles[1].buttons[0]) // engage
    {
      g_mode = MODE_ACQUIRE;
      g_acquire_start = ros::Time::now();
    }
    else
    {
      g_base_pub->publish(base_cmd);
      return;
    }
  }
  // deadman button was released. disengage
  if (!msg.paddles[1].buttons[0])
  {
    g_mode = MODE_IDLE;
    g_base_pub->publish(base_cmd);
    return;
  }
  base_cmd.linear.x = scale_and_clamp(msg.paddles[1].joy[1], 1, 0.2);
  base_cmd.linear.y = scale_and_clamp(-msg.paddles[1].joy[0], 1, 0.2);
  base_cmd.angular.z = scale_and_clamp(-msg.paddles[0].joy[0], 1, 0.4);
  g_base_pub->publish(base_cmd);

  double mix = (ros::Time::now() - g_acquire_start).toSec() / ACQUIRE_LENGTH;
  if (mix > 1)
    mix = 1;
  else if (mix < 0)
    mix = 0;

  for (int s = 0; s < 2; s++)
  {
    tf::StampedTransform transform;
    try {
      const char *target_name;
      if (s == 0)
        target_name = "/l_wrist_roll_link";
      else
        target_name = "/r_wrist_roll_link";
      g_listener->lookupTransform("/torso_lift_link", target_name,
                                  ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    double ref_orient[4] = { transform.getRotation().x(),
                             transform.getRotation().y(),
                             transform.getRotation().z(),
                             transform.getRotation().w() };
    btQuaternion q_ref(ref_orient[0], ref_orient[1],
                       ref_orient[2], ref_orient[3]);

    double pose_mix = mix;
    double orient_mix = mix;

    geometry_msgs::PoseStamped cmd;
    cmd.header.frame_id = "/torso_lift_link";
    btVector3 pose_point = transform.getOrigin().lerp(sane_pos[s], pose_mix);
    cmd.pose.position.x = pose_point.x();
    cmd.pose.position.y = pose_point.y();
    cmd.pose.position.z = pose_point.z();

    btQuaternion pose_rot;
    tf::quaternionMsgToTF(msg.paddles[s].transform.rotation, pose_rot);
    tf::quaternionTFToMsg(transform.getRotation().slerp(pose_rot, orient_mix),
                          cmd.pose.orientation);
    g_pose_pub[s]->publish(cmd);

    // TODO: add buttons to open/close gripper gradually, so you don't have to
    // hold it
    pr2_controllers_msgs::Pr2GripperCommand grip;
    grip.position = 0.08 * (1.0 - msg.paddles[s].trigger);
    grip.max_effort = 50; // no idea
    if (g_gripper_pub[s])
      g_gripper_pub[s]->publish(grip);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_pr2_hydra");
  ros::NodeHandle n, n_private("~");
  tf::TransformListener listener;
  g_listener = &listener;

  // arm IK targets
  ros::Publisher l_pose_pub = n.advertise<geometry_msgs::PoseStamped>
                              ("l_cart/command", 1);
  ros::Publisher r_pose_pub = n.advertise<geometry_msgs::PoseStamped>
                              ("r_cart/command", 1);
  g_pose_pub[0] = &l_pose_pub;
  g_pose_pub[1] = &r_pose_pub;

  // grippers
  ros::Publisher l_gripper_pub =
    n.advertise<pr2_controllers_msgs::Pr2GripperCommand>
               ("l_gripper_controller/command", 1);
  ros::Publisher r_gripper_pub =
    n.advertise<pr2_controllers_msgs::Pr2GripperCommand>
               ("r_gripper_controller/command", 1);
  g_gripper_pub[0] = &l_gripper_pub;
  g_gripper_pub[1] = &r_gripper_pub;

  // all ur base
  ros::Publisher base_pub = n.advertise<geometry_msgs::Twist>
                                       ("base_controller/command", 1);
  g_base_pub = &base_pub;

  tf::TransformBroadcaster tf_broadcaster;
  g_tf_broadcaster = &tf_broadcaster;
  ros::Subscriber hydra_sub = n.subscribe("hydra_calib", 1, hydra_cb);
  ros::spin();
  
  return 0;
}
