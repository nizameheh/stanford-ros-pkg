#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandGoal.h>
#include <teleop_controllers/JTTeleopControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <recyclerbot/GraspAction.h>
#include <recyclerbot/CylinderArray.h>
#include <sstream>

using namespace std;

enum State {INIT, DETECTION, MOVE_CLOSE, GRASP, MOVE_AWAY, DROP};
typedef actionlib::SimpleActionClient<recyclerbot::GraspAction> GraspClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class RecyclerbotMaster
{
  private:
    
  ros::NodeHandle n;
  ros::Subscriber cylinderArray_sub;
  GraspClient* graspClient;
  GripperClient* gripperClient;
  recyclerbot::GraspGoal goal;
  geometry_msgs::PoseStamped currentPose;
  pr2_controllers_msgs::Pr2GripperCommandGoal gripperGoal;
  
  ros::Publisher maxVelocity_pub;
  
  double gripper_big_effort;
  double gripper_small_effort;
  double gripperPosition;
  
  double tabletop;
  
  geometry_msgs::PoseStamped initPose;
  int category;
  
  State state;
  
  // distance between gripper pads (in meters)
  static const double GRIPPER_CLOSED = 0.05;
  static const double GRIPPER_OPEN = 0.85;
  
  static const double PRECISION_H = 0.004;
  static const double PRECISION_L = 0.08;
  
  ros::Time finishTime;
    
  public:
    
  RecyclerbotMaster(ros::NodeHandle &_n) : n(_n)
  {
    state = INIT;
    category = 0;
    finishTime = ros::Time::now();
    
    cylinderArray_sub = n.subscribe("/cylinder_array", 10, &RecyclerbotMaster::object_pose_callback, this);
    
    // publish arm velocity
    maxVelocity_pub = n.advertise<std_msgs::Float64>("velocity_saturation", 3);
    
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
		initPose.pose.position.x = 0.17;
		initPose.pose.position.y = -0.58;
		initPose.pose.position.z = 1.2;
		
		n.getParam("object_pose/orientation/x", initPose.pose.orientation.x);
		n.getParam("object_pose/orientation/y", initPose.pose.orientation.y);
		n.getParam("object_pose/orientation/z", initPose.pose.orientation.z);
		n.getParam("object_pose/orientation/w", initPose.pose.orientation.w);
		
	  n.getParam("gripper_big_effort", gripper_big_effort);
	  n.getParam("gripper_small_effort", gripper_small_effort);
		
		// move arm to bin position
		move_arm(initPose, PRECISION_L);
		
    // open gripper
    open_gripper();
    state = DETECTION;
  }
  
  void use_low_speed()
  {
  	std_msgs::Float64 maxVelocity;
  	maxVelocity.data = 0.6;
  	maxVelocity_pub.publish(maxVelocity);
  //	cout<<"use low speed"<<endl;
  }
  void use_medium_speed()
  {
  	std_msgs::Float64 maxVelocity;
  	maxVelocity.data = 0.9;
  	maxVelocity_pub.publish(maxVelocity);
  //	cout<<"use low speed"<<endl;
  }
  void use_high_speed()
  {
  	std_msgs::Float64 maxVelocity;
  	maxVelocity.data = 1.4;
  	maxVelocity_pub.publish(maxVelocity);
  //	cout<<"use high speed"<<endl;
  }
  
  void object_pose_callback(const recyclerbot::CylinderArray& msg);
  void move_gripper(double position, double effort);
  void move_gripper(double position) {move_gripper(position, gripper_small_effort);}
  void move_arm(geometry_msgs::PoseStamped& targetPose, double precision);
  void open_gripper();
  void close_gripper(double effort);
  void close_gripper() {close_gripper(gripper_small_effort);}
  void move_arm_to_bin(geometry_msgs::PoseStamped& pose);
};

void RecyclerbotMaster::open_gripper()
{
  move_gripper(GRIPPER_OPEN);
}

void RecyclerbotMaster::close_gripper(double effort)
{
//	cout<<"++++++ closing effort: "<<effort<<endl;
//	cout<<"++++++ gripper_big_effort: "<<gripper_big_effort<<endl;
  move_gripper(GRIPPER_CLOSED, effort);
}

// move gripper to a position with a max effort
void RecyclerbotMaster::move_gripper(double position, double effort)
{
  gripperGoal.command.position = position;
  gripperGoal.command.max_effort = effort;
  
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

  gripperPosition = gripperClient->getResult()->position;
    
}

void RecyclerbotMaster::move_arm_to_bin(geometry_msgs::PoseStamped& pose)
{
	double lowerDistance[3];
	lowerDistance[0] = 0.29;
	lowerDistance[1] = 0.62;
	lowerDistance[2] = 0.55;
	
	double liftDistance = 0.3;
	
  geometry_msgs::PoseStamped targetPose;
  
  // lift
  targetPose = currentPose;
  //targetPose.pose.position.z += liftDistance;
  targetPose.pose.position.z = tabletop + 0.28;
  move_arm(targetPose, PRECISION_L);
  ros::Duration(1).sleep();
  
  // plan a simple path
  geometry_msgs::Pose path2bin[3];
  
  switch (category){
  	case 0:
  		path2bin[0] = initPose.pose;
  		path2bin[0].position.y += 0.45;
  		path2bin[0].position.x += 0.02;
  		path2bin[1] = path2bin[0];
  		path2bin[1].position.z -= lowerDistance[category];
  		path2bin[2] = path2bin[0];
  		break;
  	case 1:
  		path2bin[0] = initPose.pose;
  		path2bin[1] = path2bin[0];
  		path2bin[1].position.z -= lowerDistance[category];
  		path2bin[2] = path2bin[0];
  		break;
  	case 2:
  		path2bin[0] = initPose.pose;
  		path2bin[1] = path2bin[0];
  		path2bin[1].position.x -= 0.3;
  		path2bin[1].position.z -= lowerDistance[category];
  		path2bin[2] = path2bin[0];
  		break;
  }
  
  targetPose = initPose;
  
  // move above bin
  targetPose.pose = path2bin[0];
  move_arm(targetPose, PRECISION_L);
  ros::Duration(0.5).sleep();
  
  // move down
  targetPose.pose = path2bin[1];
  move_arm(targetPose, PRECISION_L);
  ros::Duration(0.5).sleep();
  
  // open gripper
  open_gripper();
  ros::Duration(0.5).sleep();
  
  // move up
  targetPose.pose = path2bin[2];
  move_arm(targetPose, PRECISION_L);
  ros::Duration(0.5).sleep();
  
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
    tabletop = msg.cylinders[0].pose.position.z - msg.cylinders[0].height / 2;
    category = msg.cylinders[0].category;
    
    // move above the item
    targetPose.pose.position.y += 0.02;
    targetPose.pose.position.z = tabletop + 0.28;
    targetPose.pose.position.x -= 0.08;
    move_arm(targetPose, PRECISION_H);
    
    use_low_speed();
    ros::Duration(0.5).sleep();
    
    // move to the item
    targetPose.pose.position.z -= 0.12;
    targetPose.pose.position.x += 0.025;
    move_arm(targetPose, PRECISION_H);
    ros::Duration(0.5).sleep();
    targetPose.pose.position.z = msg.cylinders[0].pose.position.z;
    targetPose.pose.position.x += (0.08 - 0.02);
    move_arm(targetPose, PRECISION_H);
    
    use_high_speed();
    ros::Duration(1).sleep();
    
    // grasp
    if (category == 1) close_gripper(gripper_big_effort);
    else close_gripper(gripper_small_effort);
    
    // distinguish between glass/plastic bottle
    if (category > 0) // not a can
    {
    	if (gripperPosition > 0.068) // this is a glass bottle
    	{
    		if (category != 1)
    		{
    			close_gripper(gripper_big_effort);
    			if (gripperPosition > 0.068) category = 1;
    		}
    	}
    	else category = 2;
    }
    
    // be careful with glass bottles
    if (category == 1) use_medium_speed();
    ros::Duration(0.5).sleep();
    
    // move to bin
    move_arm_to_bin(targetPose);
    
    if (category == 1) use_high_speed();
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
