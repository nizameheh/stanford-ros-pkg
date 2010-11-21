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

enum State {INIT, DETECTION, MOVE_CLOSE, GRASP, MOVE_AWAY, DROP};
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
  pr2_controllers_msgs::Pr2GripperCommandGoal gripperGoal;
  
  geometry_msgs::PoseStamped initPose;
  double category;
  
  State state;
  
  
  
  // distance between gripper pads (in meters)
  static const double GRIPPER_CLOSED = 0.05;
  static const double GRIPPER_OPEN = 0.85;
  
  static const double PRECISION_H = 0.006;
  static const double PRECISION_L = 0.08;
  
  ros::Time finishTime;
    
  public:
    
  RecyclerbotMaster(ros::NodeHandle &_n) : n(_n)
  {
    state = INIT;
    category = 1;
    finishTime = ros::Time::now();
    
    cylinderArray_pub = n.subscribe("/cylinder_array", 10, &RecyclerbotMaster::object_pose_callback, this);
    
    graspClient = new GraspClient("grasp_server", true);
    
    std::string client_name = "r_gripper_controller/gripper_action";
    gripperClient = new GripperClient(client_name, true);
    
    ROS_INFO("Waiting for action servers to start.");
    graspClient->waitForServer(); //will wait for infinite time
    gripperClient->waitForServer(); //will wait for infinite time
    ROS_INFO("Action servers started");
    
    // move arm away
		initPose.header.frame_id = "/base_footprint";
		initPose.header.stamp = ros::Time::now();
		initPose.pose.position.x = 0.27;
		initPose.pose.position.y = -0.55;
		initPose.pose.position.z = 1.3;
		
		n.getParam("object_pose/orientation/x", initPose.pose.orientation.x);
		n.getParam("object_pose/orientation/y", initPose.pose.orientation.y);
		n.getParam("object_pose/orientation/z", initPose.pose.orientation.z);
		n.getParam("object_pose/orientation/w", initPose.pose.orientation.w);
		
		// move arm to bin position
		move_arm(initPose, PRECISION_L);
		
    // open gripper
    open_gripper();
    state = DETECTION;
  }
  void object_pose_callback(const recyclerbot::CylinderArray& msg);
  void move_gripper(double position);
  void move_arm(geometry_msgs::PoseStamped& targetPose, double precision);
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
	double lowerDistance = 0.65;
	double liftDistance = 0.3;
	
  geometry_msgs::PoseStamped targetPose;
  // lift
  targetPose = currentPose;
  targetPose.pose.position.z += liftDistance;
  move_arm(targetPose, PRECISION_L);
  ros::Duration(1).sleep();
  // to bin
  targetPose = initPose;
  move_arm(targetPose, PRECISION_L);
  ros::Duration(1).sleep();
  // move down
  targetPose.pose.position.z -= lowerDistance;
  if (category == 2) targetPose.pose.position.x -= 0.3;
  move_arm(targetPose, PRECISION_L);  
  ros::Duration(1).sleep();
  // open gripper
  open_gripper();
  ros::Duration(1).sleep();
  // move up
  if (category == 2) targetPose.pose.position.x += 0.3;
  targetPose.pose.position.z += lowerDistance;
  move_arm(targetPose, PRECISION_L);
  ros::Duration(1).sleep();
  
}

void RecyclerbotMaster::move_gripper(double position)
{
  gripperGoal.command.position = position;
  n.getParam("gripper_max_effort", gripperGoal.command.max_effort);
  
  ROS_INFO("Gripper action server sending goal.");
  gripperClient->sendGoal(gripperGoal);
  bool finished_before_timeout = gripperClient->waitForResult(ros::Duration(5.0));
  
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripperClient->getState();
    ROS_INFO("Gripper movement finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Gripper movement did not finish before the time out.");
    
}

void RecyclerbotMaster::move_arm(geometry_msgs::PoseStamped& targetPose, double precision)
{
  ROS_INFO("Arm action server sending goal.");
  
  // send a goal to the action
  goal.targetPose = targetPose;
  goal.precision = precision;
  graspClient->sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = graspClient->waitForResult(ros::Duration(5.0));

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
  if ((state == DETECTION) && (msg.header.stamp > finishTime))
  {
    state = MOVE_CLOSE;

    open_gripper();

    geometry_msgs::PoseStamped targetPose;
    targetPose.header.frame_id = msg.header.frame_id;
    targetPose.header.stamp = ros::Time::now();  
    targetPose.pose = msg.cylinders[0].pose;
    category = msg.cylinders[0].category;
    
    // move above the item
    targetPose.pose.position.y += 0.01;
    targetPose.pose.position.z += 0.15;
    targetPose.pose.position.x -= 0.05;
    move_arm(targetPose, PRECISION_H);
    ros::Duration(1).sleep();
    // move to the item
    targetPose.pose.position.x += 0.05;
    targetPose.pose.position.x += 0.01;
    targetPose.pose.position.z -= 0.15;
    move_arm(targetPose, PRECISION_H);
    /*
    targetPose.pose.position.x -= 0.15;
    move_arm(targetPose);
    ros::Duration(2).sleep();
    targetPose.pose.position.x += 0.1;
    move_arm(targetPose);*/
    ros::Duration(1).sleep();
    close_gripper();
    ros::Duration(1).sleep();
    move_arm_to_bin(targetPose);
    state = DETECTION;
    finishTime = ros::Time::now();
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
