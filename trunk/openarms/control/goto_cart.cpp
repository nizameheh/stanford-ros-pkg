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
#include <Eigen/SVD>
#include "robot_state_publisher/treefksolverposfull_recursive.hpp"
#include <string>
#include <map>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jacobian.hpp>
#include "geometry_msgs/Transform.h"
#include <cstdio>
using std::string;
using std::map;
using std::make_pair;
using namespace Eigen;
using namespace std;

sensor_msgs::JointState g_js, g_actual_js;
ros::Publisher *g_joint_pub = NULL;
tf::TransformBroadcaster *g_tf_broadcaster = NULL;
tf::TransformListener *g_tf_listener = NULL;
KDL::TreeFkSolverPosFull_recursive *g_fk_solver = NULL;
KDL::ChainIkSolverPos_NR_JL *g_ik_solver = NULL;
KDL::ChainJntToJacSolver *g_jac_solver = NULL;
tf::Transform /*g_target_origin, */g_target;
std::vector<double> g_pose;
double g_posture_gain = 0.1;

tf::Transform fk_tool(const std::vector<double> &x) // joint angles 
{
  map<string, double> joint_pos;
  joint_pos.insert(make_pair("shoulder1", x[0]));
  joint_pos.insert(make_pair("shoulder2", x[1]));
  joint_pos.insert(make_pair("shoulder3", x[2]));
  joint_pos.insert(make_pair("elbow", x[3]));
  joint_pos.insert(make_pair("wrist1", x[4]));
  joint_pos.insert(make_pair("wrist2", x[5]));
  joint_pos.insert(make_pair("wrist3", x[6]));
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
  /*
  const double POSE_GAIN = 0.1;
  q_init.data[1] += POSE_GAIN * (0.7 - joints[1]);
  q_init.data[2] += POSE_GAIN * (0 - joints[2]);
  q_init.data[3] += POSE_GAIN * (-0.1 - joints[3]);
  q_init.data[4] += POSE_GAIN * (0 - joints[4]);
  */
  // populate F_dest from tf::Transform parameter
  KDL::Frame F_dest;
  tf::TransformTFToKDL(t, F_dest);
  if (g_ik_solver->CartToJnt(q_init, F_dest, q) < 0)
  {
    ROS_ERROR("ik solver fail");
    return false;
  }

  KDL::Jacobian jac(7);
  g_jac_solver->JntToJac(q, jac);
  printf("jacobian: %d x %d\n", jac.data.transpose().rows(), jac.data.transpose().cols());
  //SVD< Eigen::Matrix<double, 7, Eigen::Dynamic> > svd(jac.data.transpose());
  MatrixXd jac_padded(7, 7);
  for (int i = 0; i < 7; i++)
    for (int j = 0; j < 7; j++)
    {
      if (j < 6)
        jac_padded(i, j) = jac(j, i);
      else
        jac_padded(i, j) = 0;
    }
  SVD< Eigen::MatrixXd > svd(jac_padded);
  printf("%d singular values\n", svd.singularValues().size());
  for (int i = 0; i < svd.singularValues().size(); i++)
    printf("%d:  %f\n", i, svd.singularValues()(i));

  //cout << jac.data << endl << endl;
  cout << svd.matrixU() << endl << endl;

  for (int i = 0; i < 7; i++)
    joints[i] = q.data[i] - g_posture_gain * svd.matrixU()(i, 6);
  return true;
}

void joint_cb(const sensor_msgs::JointState &msg)
{
  g_actual_js = msg;
}

void target_cb(const geometry_msgs::Transform &t_msg)
{
  // assume the transform coming in is a delta away from the center 
  // of our workspace (for now, at least)
  //tf::Transform t_bump(btQuaternion::getIdentity(),
  //                     btVector3(d_pose, 0, 0));
  //tf::Transform t_target = t_tool * t_bump;
  tf::Transform t_input;
  tf::transformMsgToTF(t_msg, t_input);
  tf::StampedTransform t(t_input, ros::Time::now(), "world", "ik_target");
  /*
  tf::StampedTransform t_target_origin(g_target_origin, ros::Time::now(),
                                         "torso_link", "target_origin");
  */
  std::vector<double> j_ik;// = g_pose;
  j_ik.resize(7);
  if (g_actual_js.position.size() >= 7)
  {
    for (int i = 0; i < 7; i++)
      j_ik[i] = g_actual_js.position[i];
  }
  //t.setRotation(t.getRotation() * btQuaternion(0, 0, 0));
  //t_target_in_torso = g_target_origin * t;
  /*
  try
  {
    g_tf_listener->lookupTransform("torso_link", "human_tool_link",
                                   ros::Time(0), t);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  */
  tf::StampedTransform t_target_in_torso(t, ros::Time::now(),
                                         "world", "ik_target");
  geometry_msgs::TransformStamped target_trans_msg;
  tf::transformStampedTFToMsg(t_target_in_torso, target_trans_msg);
  g_tf_broadcaster->sendTransform(target_trans_msg);

/*

  */
  /*
  target_trans.header.frame_id = "torso_link";
  target_trans.header.stamp = ros::Time::now();
  target_trans.child_frame_id = "ik_target";
  */
  //target_trans.transform.translation = g_target_origin * t;
/*
*/
/*
  tf::StampedTransform t_target_origin(g_target_origin, ros::Time::now(),
                                         "torso_link", "target_origin");
  geometry_msgs::TransformStamped target_origin_msg;
  tf::transformStampedTFToMsg(t_target_origin, target_origin_msg);
  g_tf_broadcaster->sendTransform(target_origin_msg);
*/
//  ik_tool(g_target_origin * t, j_ik);
  ik_tool(t, j_ik);

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
  ros::Subscriber joint_sub = n.subscribe("joint_states", 1, joint_cb);
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("target_joints", 1);
  g_joint_pub = &joint_pub; // ugly ugly
  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformListener tf_listener;
  g_tf_broadcaster = &tf_broadcaster;
  g_tf_listener = &tf_listener;
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
  g_js.name[0] = "shoulder1";
  g_js.name[1] = "shoulder2`";
  g_js.name[2] = "shoulder3";
  g_js.name[3] = "elbow";
  g_js.name[4] = "wrist1";
  g_js.name[5] = "wrist2";
  g_js.name[6] = "wrist3";

  KDL::TreeFkSolverPosFull_recursive fk_solver(tree);
  g_fk_solver = &fk_solver;
  KDL::SegmentMap::const_iterator root_seg = tree.getRootSegment();
  string tree_root_name = root_seg->first;
  printf("root: %s\n", tree_root_name.c_str());
  KDL::ChainFkSolverPos_recursive fk_solver_chain(chain);
  KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
  KDL::JntArray q_min(7), q_max(7);
  q_min.data[0] = -.7;
  q_min.data[1] = -2.5;
  q_min.data[2] = -2;
  q_min.data[3] = -1.6;
  q_min.data[4] = -3.14;
  q_min.data[5] = -1.57;
  q_min.data[6] = -3.14;
  q_max.data[0] = 1.0;
  q_max.data[1] = 0.0;
  q_max.data[2] = 2;
  q_max.data[3] = 1.0;
  q_max.data[4] = 3.14;
  q_max.data[5] = 1.57;
  q_max.data[6] = 3.14;
  KDL::ChainIkSolverPos_NR_JL ik_solver_pos(chain, q_min, q_max,
                                            fk_solver_chain, ik_solver_vel, 
                                            100, 1e-6);
  //KDL::ChainIkSolverPos_NR ik_solver_pos(chain, fk_solver_chain, ik_solver_vel, 100, 1e-6);
  g_ik_solver = &ik_solver_pos;

  KDL::ChainJntToJacSolver jac_solver(chain);
  g_jac_solver = &jac_solver;

  //boost::scoped_ptr<KDL::TreeFkSolverPosFull_recursive> fk_solver;

  g_pose.resize(7);
  g_pose[0] = .3;
  g_pose[1] = -.3;
  g_pose[2] = -.6;
  g_pose[3] = 0;
  g_pose[4] = 0;
  g_pose[5] = 0;
  g_pose[6] = 0;
  //tf::Transform t_tool = fk_tool(g_pose);
  //double d_pose = 0, d_pose_inc = 0.01;

  // set origin to be something we can comfortably reach
  //g_target_origin = fk_tool(g_pose);
  //btQuaternion target_quat;
  //target_quat.setEuler(1.57, -1.57, 0);
  //g_target_origin.setRotation(target_quat);
  //g_target_origin = btTransform(btQuaternion::getIdentity(), btVector3(0, 0.1, 0));
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

