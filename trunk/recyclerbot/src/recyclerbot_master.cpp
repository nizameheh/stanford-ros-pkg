#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandGoal.h>
#include <teleop_controllers/JTTeleopControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <recyclerbot/GraspAction.h>
#include <recyclerbot/CylinderArray.h>

using namespace std;

enum State {DETECTION, MOVE_CLOSE, GRASP, MOVE_AWAY, DROP};
typedef actionlib::SimpleActionClient<recyclerbot::GraspAction> GraspClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class RecyclerbotMaster
{
  private:
    
  ros::NodeHandle n;
  ros::Subscriber cylinderArray_pub;
  GraspClient* graspClient;
  GripperClient* gripperClient;
  recyclerbot::GraspGoal goal;
  geometry_msgs::PoseStamped currentPose;
  
  State state;
  
  // distance between gripper pads (in meters)
  static const double GRIPPER_CLOSED = 0.05;
  static const double GRIPPER_OPEN = 0.85;
  static const double GRIPPER_OBJECT_PRESENCE_THRESHOLD = 0.001;
    
  public:
    
  RecyclerbotMaster(ros::NodeHandle &_n) : n(_n)
  {
    state = DETECTION;
    cylinderArray_pub = n.subscribe("/cylinder_array", 10, &RecyclerbotMaster::object_pose_callback, this);
    
    graspClient = new GraspClient("grasp_server", true);
    
    std::string client_name = "r_gripper_controller/gripper_action";
    gripperClient = new GripperClient(client_name, true);
    
    ROS_INFO("Waiting for action server to start.");
    graspClient->waitForServer(); //will wait for infinite time
    gripperClient->waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started");
    
    // open gripper
    open_gripper();
  }
  void object_pose_callback(const recyclerbot::CylinderArray& msg);
  void move_gripper(double position);
  void move_arm(geometry_msgs::PoseStamped& targetPose);
  void open_gripper();
  void close_gripper();
  void move_arm_to_bin(geometry_msgs::PoseStamped& pose);
};

void RecyclerbotMaster::open_gripper()
{
  move_gripper(GRIPPER_OPEN);
}

void RecyclerbotMaster::close_gripper()
{
  move_gripper(GRIPPER_CLOSED);
}

void RecyclerbotMaster::move_arm_to_bin(geometry_msgs::PoseStamped& pose)
{
  geometry_msgs::PoseStamped targetPose;
  targetPose = currentPose;
  targetPose.pose.position.z += 0.2;
  move_arm(targetPose);
  ros::Duration(1).sleep();
  targetPose.pose.position.x -= 0.2;
  targetPose.pose.position.y -= 0.5;
  move_arm(targetPose);
  ros::Duration(1).sleep();
}

void RecyclerbotMaster::move_gripper(double position)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal gripperGoal;
  gripperGoal.command.position = position;
  gripperGoal.command.max_effort = 40;
  
  ROS_INFO("Gripper action server sending goal.");
  gripperClient->sendGoal(gripperGoal);
  bool finished_before_timeout = gripperClient->waitForResult(ros::Duration(10.0));
  
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripperClient->getState();
    ROS_INFO("Gripper movement finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Gripper movement did not finish before the time out.");
    
}

void RecyclerbotMaster::move_arm(geometry_msgs::PoseStamped& targetPose)
{
  ROS_INFO("Arm action server sending goal.");
  
  // send a goal to the action
  goal.targetPose = targetPose;
  graspClient->sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = graspClient->waitForResult(ros::Duration(10.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = graspClient->getState();
    currentPose = graspClient->getResult()->endPose;
    ROS_INFO("Arm movement finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Arm movement did not finish before the time out.");
}

void RecyclerbotMaster::object_pose_callback(const recyclerbot::CylinderArray& msg)
{
  if (state == DETECTION)
  {
    state = MOVE_CLOSE;

    geometry_msgs::PoseStamped targetPose;
    targetPose.header.frame_id = msg.header.frame_id;
    targetPose.header.stamp = ros::Time::now();  
    targetPose.pose = msg.cylinders[0].pose;
    
    targetPose.pose.position.x -= 0.15;
    move_arm(targetPose);
    ros::Duration(2).sleep();
    targetPose.pose.position.x += 0.1;
    move_arm(targetPose);
    ros::Duration(5).sleep();
    close_gripper();
    ros::Duration(2).sleep();
    move_arm_to_bin(targetPose);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recyclerbot_master");
  ros::NodeHandle n;
  
  RecyclerbotMaster recyclerbotMaster(n);
  ros::spin();
  
  return 0;
}
