// bsd license blah blah
#include <cstring>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include "openarms/ArmSensors.h"
#include "wiimote/State.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "robot_state_publisher/treefksolverposfull_recursive.hpp"
#include <string>
#include <map>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "geometry_msgs/Transform.h"
using std::string;
using std::map;
using std::make_pair;
using namespace Eigen;

sensor_msgs::JointState g_js;
ros::Publisher *g_joint_pub = NULL;
tf::TransformBroadcaster *g_tf_broadcaster = NULL;
KDL::TreeFkSolverPosFull_recursive *g_fk_solver = NULL;
KDL::ChainIkSolverPos_NR *g_ik_solver = NULL;
tf::Transform g_target_origin, g_target;
std::vector<double> g_pose;


tf::Transform fk_tool(const std::vector<double> &x) // joint angles 
{
  map<string, double> joint_pos;
  joint_pos.insert(make_pair("shoulder_flexion", x[0]));
  joint_pos.insert(make_pair("shoulder_abduction", x[1]));
  joint_pos.insert(make_pair("humeral_rotation", x[2]));
  joint_pos.insert(make_pair("elbow", x[3]));
  joint_pos.insert(make_pair("forearm_roll", x[4]));
  joint_pos.insert(make_pair("wrist_pitch", x[5]));
  joint_pos.insert(make_pair("wrist_roll", x[6]));
  map<string, KDL::Frame> link_poses;
  g_fk_solver->JntToCart(joint_pos, link_poses);
  if (link_poses.size() < 7)
    ROS_ERROR("couldn't compute forward kinematics.");
  for (map<string, KDL::Frame>::const_iterator f = link_poses.begin();
      f != link_poses.end(); ++f)
  {
    if (f->first == "tool_link")
    {
      tf::Transform tf_frame;
      tf::TransformKDLToTF(f->second, tf_frame);
      return tf_frame;
    }
  }
  return tf::Transform();
}

bool ik_tool(tf::Transform t, std::vector<double> &joints)
{
  // assumes that "joints" coming in is the initial vector
  if (joints.size() != 7)
  {
    ROS_ERROR("woah there. the joints vector is supposed to be the start pos");
    return false;
  }
  KDL::JntArray q_init(7), q(7);
  for (int i = 0; i < 7; i++)
    q_init.data[i] = joints[i];
  // populate F_dest from tf::Transform parameter
  KDL::Frame F_dest;
  tf::TransformTFToKDL(t, F_dest);
  if (g_ik_solver->CartToJnt(q_init, F_dest, q) < 0)
  {
    ROS_ERROR("ik solver fail");
    return false;
  }
  for (int i = 0; i < 7; i++)
    joints[i] = q.data[i];
  return true;
}

void target_cb(const geometry_msgs::Transform &t_msg)
{
  // assume the transform coming in is a delta away from the center 
  // of our workspace (for now, at least)
  //tf::Transform t_bump(btQuaternion::getIdentity(),
  //                     btVector3(d_pose, 0, 0));
  //tf::Transform t_target = t_tool * t_bump;
  tf::Transform t;
  tf::transformMsgToTF(t_msg, t);
  std::vector<double> j_ik = g_pose;
  ik_tool(g_target_origin * t, j_ik);
  g_js.header.stamp = ros::Time::now();
  g_js.position = j_ik;
  g_joint_pub->publish(g_js);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ik_test");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  string robot_desc;
  n.getParam("robot_description", robot_desc);
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(robot_desc, tree))
  {
    ROS_ERROR("failed to extract kdl tree from xml robot description");
    return 1;
  }
  if (!tree.getNrOfSegments())
  {
    ROS_ERROR("empty tree. sad.");
    return 1;
  }
  KDL::Chain chain;
  if (!tree.getChain("torso_link", "tool_link", chain))
  {
    ROS_ERROR("couldn't pull arm chain from robot model");
    return 1;
  }
  ROS_INFO("parsed tree successfully");
  ///////////////////////////////////////////////////////////////////////

  ros::Subscriber target_sub = n.subscribe("target_frame", 1, target_cb);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  g_joint_pub = &joint_pub; // ugly ugly
  tf::TransformBroadcaster tf_broadcaster;
  g_tf_broadcaster = &tf_broadcaster;
  ros::Rate loop_rate(20);
  geometry_msgs::TransformStamped world_trans;
  world_trans.header.frame_id = "world";
  world_trans.child_frame_id = "torso_link";
  world_trans.transform.translation.x = 0;
  world_trans.transform.translation.y = 0;
  world_trans.transform.translation.z = 0;
  world_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

  g_js.name.resize(7);
  g_js.position.resize(7);
  g_js.name[0] = "shoulder_flexion";
  g_js.name[1] = "shoulder_abduction";
  g_js.name[2] = "humeral_rotation";
  g_js.name[3] = "elbow";
  g_js.name[4] = "forearm_roll";
  g_js.name[5] = "wrist_pitch";
  g_js.name[6] = "wrist_roll";

  KDL::TreeFkSolverPosFull_recursive fk_solver(tree);
  g_fk_solver = &fk_solver;
  KDL::SegmentMap::const_iterator root_seg = tree.getRootSegment();
  string tree_root_name = root_seg->first;
  printf("root: %s\n", tree_root_name.c_str());
  KDL::ChainFkSolverPos_recursive fk_solver_chain(chain);
  KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
  KDL::ChainIkSolverPos_NR ik_solver_pos(chain, fk_solver_chain, ik_solver_vel, 100, 1e-6);
  g_ik_solver = &ik_solver_pos;

  //boost::scoped_ptr<KDL::TreeFkSolverPosFull_recursive> fk_solver;

  g_pose.resize(7);
  g_pose[0] = .3;
  g_pose[1] = -.6;
  g_pose[2] = -0.4;
  g_pose[3] = 0;
  g_pose[4] = -0.4;
  g_pose[5] = 0;
  g_pose[6] = -0.4;
  //tf::Transform t_tool = fk_tool(g_pose);
  //double d_pose = 0, d_pose_inc = 0.01;

  // set origin to be something we can comfortably reach
  g_target_origin = fk_tool(g_pose);
  //tf::Transform(btQuaternion::getIdentity(), btVector3(0, 0, 0));

  while (ros::ok())
  {
    world_trans.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(world_trans);

/*
    double x = t.getOrigin().x(), y = t.getOrigin().y(), z = t.getOrigin().z();
    double roll, pitch, yaw;
    btMatrix3x3(t.getRotation()).getRPY(roll, pitch, yaw);
    printf("%.3f %.3f %.3f   %.3f %.3f %.3f\n", x, y, z, roll, pitch, yaw);
*/
    //std::vector<double> j_ik = pose;
    //js.position[2] += 1;
    //j_ik = pose;js.position;
    //printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
    //       j_ik[0], j_ik[1], j_ik[2], j_ik[3], j_ik[4], j_ik[5], j_ik[6]);
    //j_ik[2] += posture;
    //j_ik[3] += posture;
    //j_ik[4] += posture;
#if 0
    tf::Transform t_bump(btQuaternion::getIdentity(),
                         btVector3(d_pose, 0, 0));
    //tf::Transform t_target = t_tool * t_bump;

    ik_tool(t_tool * t_bump, j_ik);
#endif


    //double x = t.getOrigin().x(), y = t.getOrigin().y(), z = t.getOrigin().z();
    /*
    printf("%.3f   %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
           posture,
           j_ik[0], j_ik[1], j_ik[2], j_ik[3], j_ik[4], j_ik[5], j_ik[6]);
    printf("\n");
    */
/*
    js.header.stamp = ros::Time::now();
    js.position = j_ik;
    joint_pub.publish(js);

    d_pose += d_pose_inc;
    if (d_pose > 0.28)
      d_pose_inc = -0.005;
    else if (d_pose < -0.30)
      d_pose_inc = 0.005;
    printf("%f %f\n", d_pose, d_pose_inc);
*/
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

