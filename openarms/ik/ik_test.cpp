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
using std::string;
using std::map;
using std::make_pair;
using namespace Eigen;

// (MQ:) much of the interface code between ROS and KDL in this code is lifted
// straight from Wim Meeussen's robot_state_publisher
#if 0
class EKF
{
public:
  static const int NUM_JOINTS = 4;
  static const int N = NUM_JOINTS * 2; // joint positions and velocities
  static const int K = 12; // 2 wiimotes, 6-dof each
  VectorXd mean;
  MatrixXd cov, transition_cov, meas_cov;
  //tf::TransformListener tf_listener;
  tf::StampedTransform upperarm_tf, lowerarm_tf;
  KDL::Tree *tree;
  boost::scoped_ptr<KDL::TreeFkSolverPosFull_recursive> fk_solver;
  std::string tree_root_name;

  EKF(KDL::Tree *init_tree) : tree(init_tree)
  {
    fk_solver.reset(new KDL::TreeFkSolverPosFull_recursive(*tree));
    KDL::SegmentMap::const_iterator root_seg = tree->getRootSegment();
    tree_root_name = root_seg->first;
  }

  void init(const VectorXd &init_mean, const MatrixXd &init_cov,
            const MatrixXd &init_transition_cov,
            const MatrixXd &init_meas_cov)
  {
    mean = init_mean;
    cov = init_cov;
    transition_cov = init_transition_cov;
    meas_cov = init_meas_cov;
  }
  Matrix4d construct_transform(const Matrix3d &rot, const Vector3d &vec)
  {
    Matrix4d T_matrix;
    T_matrix.block(0, 0, 3, 3) = rot;
    T_matrix.block(0, 3, 3, 1) = vec;
    T_matrix(3, 0) = T_matrix(3, 1) = T_matrix(3, 2) = 0;
    T_matrix(3, 3) = 1;
    return T_matrix;
  }
  VectorXd f(const VectorXd &x) // state transition
  {
    const double SENSOR_RATE = 66.667; // appears to be what the wiimote gives
    VectorXd x_f(N);
    for (int i = 0; i < NUM_JOINTS; i++)
    {
      x_f[i] = x[i] + 1. / SENSOR_RATE * x[i+NUM_JOINTS]; // newton was genius
      x_f[i] *= 0.99; // decay to zero 
    }
    for (int i = NUM_JOINTS; i < 2*NUM_JOINTS; i++)
      x_f[i] = x[i]; // assume constant velocity
    return x_f;
  }
  bool update_tf()
  {
    /*
    try
    {
      tf_listener.lookupTransform("/upperarm_wiimote_link", "/world",
                                  ros::Time(0), upperarm_tf);
      tf_listener.lookupTransform("/lowerarm_wiimote_link", "/world",
                                  ros::Time(0), lowerarm_tf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    btVector3 accel = 9.81 * upperarm_tf.getBasis().getColumn(2);
    ROS_INFO("predicted upperarm: %+05.2f %+05.2f %+05.2f\n",
             accel.x(), accel.y(), accel.z());
    return true;
    */
    return true;
  }
  VectorXd z(const VectorXd &x)
  {
    map<string, double> joint_pos;
    joint_pos.insert(make_pair("shoulder_flexion", x[0]));
    joint_pos.insert(make_pair("shoulder_abduction", x[1]));
    joint_pos.insert(make_pair("humeral_rotation", x[2]));
    joint_pos.insert(make_pair("elbow", x[3]));
    map<string, KDL::Frame> link_poses;
    fk_solver->JntToCart(joint_pos, link_poses);
    VectorXd pred_sensors = VectorXd::Zero(12);
    if (link_poses.size() < 4)
      ROS_ERROR("couldn't compute link poses.");
    for (map<string, KDL::Frame>::const_iterator f = link_poses.begin();
         f != link_poses.end(); ++f)
    {
      if (f->first == "upperarm_wiimote_link")
      {
        tf::Transform tf_frame;
        tf::TransformKDLToTF(f->second, tf_frame);
        btVector3 accel = tf_frame.getBasis().getRow(2);
        pred_sensors[0] = accel.x();
        pred_sensors[1] = accel.y();
        pred_sensors[2] = accel.z();
        /*
        ROS_INFO("predicted upperarm: %+05.2f %+05.2f %+05.2f",
                 accel.x(), accel.y(), accel.z());
        */
      }
      else if (f->first == "lowerarm_wiimote_link")
      {
        tf::Transform tf_frame;
        tf::TransformKDLToTF(f->second, tf_frame);
        btVector3 accel = tf_frame.getBasis().getRow(2);
        pred_sensors[3] = accel.x();
        pred_sensors[4] = accel.y();
        pred_sensors[5] = accel.z();
        /*
        ROS_INFO("predicted lowerarm: %+05.2f %+05.2f %+05.2f",
                 accel.x(), accel.y(), accel.z());
        */
      }
    }
    return pred_sensors;
  }
  void update(const VectorXd &meas)
  {
    const double DELTA = 0.0001; // numerical derivatives
    // state update ////////////////////////////////////////////////////////
    // predict the next mean
    VectorXd pred_mean = f(mean);
    // predict covariance: compute linearization of transition function at mean
    MatrixXd A(N, N);
    for (int j = 0; j < N; j++)
    {
      VectorXd bumped = mean;
      bumped(j) += DELTA;
      VectorXd bumped_next = f(bumped);
      A.col(j) = (bumped_next - pred_mean) / DELTA;
    }
    MatrixXd pred_cov = A * cov * A.transpose() + transition_cov;
    // measurement update //////////////////////////////////////////////////
    // compute linearization C of the measurement function z at predicted mean
    VectorXd pred_meas = z(pred_mean);
    MatrixXd C(K, N);
    for (int j = 0; j < N; j++)
    {
      VectorXd bumped = pred_mean;
      bumped(j) += DELTA;
      VectorXd bumped_meas = z(bumped);
      C.col(j) = (bumped_meas - pred_meas) / DELTA;
    }
    // compute kalman gain
    MatrixXd t = C * pred_cov * C.transpose() + meas_cov;
    MatrixXd KG = pred_cov * C.transpose() * t.inverse();
    mean = pred_mean + KG * (meas - pred_meas);
    cov = (MatrixXd::Identity(N, N) - KG * C) * pred_cov;
  }
};
#endif

ros::Publisher *g_joint_pub = NULL;
tf::TransformBroadcaster *g_tf_broadcaster = NULL;
KDL::TreeFkSolverPosFull_recursive *g_fk_solver = NULL;
KDL::ChainIkSolverPos_NR *g_ik_solver = NULL;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ik_test");
  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  ///////////////////////////////////////////////////////////////////////
  // this code lifted from the robot_state_publisher package
  string robot_desc;
  n.getParam("robot_description", robot_desc);
  //ROS_INFO("robot: %s", robot_desc.c_str());
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

  //n_private.getParam("joint0bias", g_joint_bias[0]);
  //n_private.getParam("joint1bias", g_joint_bias[1]);
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

  sensor_msgs::JointState js;
  js.name.resize(7);
  js.position.resize(7);
  js.name[0] = "shoulder_flexion";
  js.name[1] = "shoulder_abduction";
  js.name[2] = "humeral_rotation";
  js.name[3] = "elbow";
  js.name[4] = "forearm_roll";
  js.name[5] = "wrist_pitch";
  js.name[6] = "wrist_roll";

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

  std::vector<double> pose;
  pose.resize(7);
  pose[0] = .3;
  pose[1] = -.6;
  pose[2] = -0.4;
  pose[3] = 0;
  pose[4] = -0.4;
  pose[5] = 0;
  pose[6] = -0.4;
  tf::Transform t_tool = fk_tool(pose);
  double d_pose = 0, d_pose_inc = 0.01;

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
    std::vector<double> j_ik = pose;
    //js.position[2] += 1;
    //j_ik = pose;js.position;
    //printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
    //       j_ik[0], j_ik[1], j_ik[2], j_ik[3], j_ik[4], j_ik[5], j_ik[6]);
    //j_ik[2] += posture;
    //j_ik[3] += posture;
    //j_ik[4] += posture;
    tf::Transform t_bump(btQuaternion::getIdentity(),
                         btVector3(d_pose, 0, 0));
    //tf::Transform t_target = t_tool * t_bump;
    ik_tool(t_tool * t_bump, j_ik);
    //double x = t.getOrigin().x(), y = t.getOrigin().y(), z = t.getOrigin().z();
    /*
    printf("%.3f   %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
           posture,
           j_ik[0], j_ik[1], j_ik[2], j_ik[3], j_ik[4], j_ik[5], j_ik[6]);
    printf("\n");
    */

    js.header.stamp = ros::Time::now();
    js.position = j_ik;
    joint_pub.publish(js);

    d_pose += d_pose_inc;
    if (d_pose > 0.28)
      d_pose_inc = -0.005;
    else if (d_pose < -0.30)
      d_pose_inc = 0.005;
    printf("%f %f\n", d_pose, d_pose_inc);

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

