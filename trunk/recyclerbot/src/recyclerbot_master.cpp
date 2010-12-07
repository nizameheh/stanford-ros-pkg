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

#define RIGHT_ARM 0
#define LEFT_ARM 1

using namespace std;

enum State {INIT, DETECTION, MOVE_CLOSE, GRASP, MOVE_AWAY, DROP};
typedef actionlib::SimpleActionClient<recyclerbot::GraspAction> GraspClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class RecyclerbotMaster
{
  private:
    
  ros::NodeHandle n;
  ros::Subscriber cylinderArray_sub;
  GraspClient* graspClientR;
  GripperClient* gripperClientR;
  GraspClient* graspClientL;
  GripperClient* gripperClientL;
  recyclerbot::GraspGoal goal;
  geometry_msgs::PoseStamped currentPose;
  pr2_controllers_msgs::Pr2GripperCommandGoal gripperGoal;
  
  ros::Publisher maxVelocityR_pub;
  ros::Publisher maxVelocityL_pub;
  
  double gripper_big_effort;
  double gripper_small_effort;
  double max_squeeze_effort;
  double min_squeeze_effort;
  double gripperPosition;
  
  double tabletop;
  
  geometry_msgs::PoseStamped initPoseR;
  geometry_msgs::PoseStamped initPoseL;
  int category;
  
  State state;
  
  // distance between gripper pads (in meters)
  double gripperOpen;
  double gripperClose;
  
  double armPrecisionH;
  double armPrecisionL;
  double armTimeout;
  double gripperTimeout;
  
  ros::Time finishTime;
    
  public:
    
  RecyclerbotMaster(ros::NodeHandle &_n) : n(_n)
  {
    state = INIT;
    category = 0;
    finishTime = ros::Time::now();
    
    cylinderArray_sub = n.subscribe("/cylinder_array", 10, &RecyclerbotMaster::object_pose_callback, this);
    
    // publish arm max velocity
    maxVelocityR_pub = n.advertise<std_msgs::Float64>("r_velocity_saturation", 3);
    maxVelocityL_pub = n.advertise<std_msgs::Float64>("l_velocity_saturation", 3);
    
    // setup arm and gripper servers
    graspClientR = new GraspClient("r_grasp_server", true);
    graspClientL = new GraspClient("l_grasp_server", true);
    
    std::string clientNameR = "r_gripper_controller/gripper_action";
    gripperClientR = new GripperClient(clientNameR, true);
    std::string clientNameL = "l_gripper_controller/gripper_action";
    gripperClientL = new GripperClient(clientNameL, true);
    
    ROS_INFO("Waiting for action servers to start.");
    graspClientR->waitForServer(); //will wait for infinite time
    gripperClientR->waitForServer(); //will wait for infinite time
    graspClientL->waitForServer(); //will wait for infinite time
    gripperClientL->waitForServer(); //will wait for infinite time
    ROS_INFO("Action servers started");
    
    // initial pose of arm (should be out of sight)
		initPoseR.header.frame_id = "/base_footprint";
		initPoseR.header.stamp = ros::Time::now();
		
		n.param("arm_init_pose/position/x", initPoseR.pose.position.x, 0.17);
		n.param("arm_init_pose/position/y", initPoseR.pose.position.y, -0.58);
		n.param("arm_init_pose/position/z", initPoseR.pose.position.z, 1.2);
		n.param("arm_init_pose/orientation/x", initPoseR.pose.orientation.x, 0.0);
		n.param("arm_init_pose/orientation/y", initPoseR.pose.orientation.y, 0.3);
		n.param("arm_init_pose/orientation/z", initPoseR.pose.orientation.z, 0.0);
		n.param("arm_init_pose/orientation/w", initPoseR.pose.orientation.w, 0.953939201);
		
		initPoseL = initPoseR;
		initPoseL.pose.position.y = 0.5;//-initPoseR.pose.position.y;
		initPoseL.header.stamp = ros::Time::now();
		
		// load parameters
	  n.param("gripper_big_effort", gripper_big_effort, 50.0);
	  n.param("gripper_small_effort", gripper_small_effort, 17.0);
	  n.param("max_squeeze_effort", max_squeeze_effort, 50.0);
	  n.param("min_squeeze_effort", min_squeeze_effort, 10.0);
	  n.param("high_arm_move_precision", armPrecisionH, 0.004);
	  n.param("low_arm_move_precision", armPrecisionL, 0.08);
	  n.param("gripper_open_position", gripperOpen, 0.085);
	  n.param("gripper_close_position", gripperClose, 0.03);
	  n.param("arm_move_timeout", armTimeout, 5.0);
	  n.param("gripper_move_timeout", gripperTimeout, 5.0);
		
		// move arms to initial position
		move_arm(initPoseR, armPrecisionL, RIGHT_ARM);
		move_arm(initPoseL, armPrecisionL, LEFT_ARM);
		
    // open grippers
    open_gripper(RIGHT_ARM);
    open_gripper(LEFT_ARM);
    
    state = DETECTION;
  }
  
  void use_low_speed(int whichArm)
  {
  	set_arm_speed(0.4, whichArm);
  }
  void use_medium_speed(int whichArm)
  {
  	set_arm_speed(0.9, whichArm);
  }
  void use_high_speed(int whichArm)
  {
  	set_arm_speed(1.4, whichArm);
  }
  void set_arm_speed(double speed, int whichArm)
  {  	
  	ros::Publisher* maxVelocity_pub = (whichArm == RIGHT_ARM) ? &maxVelocityR_pub : &maxVelocityL_pub;
  	std_msgs::Float64 maxVelocity;
  	maxVelocity.data = speed;
  	maxVelocity_pub->publish(maxVelocity);
  	cout<<"setting arm speed to: "<<speed<<endl;
  }
  
  void object_pose_callback(const recyclerbot::CylinderArray& msg);
  void move_gripper(double position, double effort, int whichArm);
  void move_gripper(double position, int whichArm) {move_gripper(position, gripper_small_effort, whichArm);}
  void move_arm(geometry_msgs::PoseStamped& targetPose, double precision, int whichArm);
  void open_gripper(int whichArm);
  void close_gripper(double effort, int whichArm);
  void close_gripper(int whichArm) {close_gripper(gripper_small_effort, whichArm);}
  void move_arm_to_bin(geometry_msgs::PoseStamped& pose, int whichArm);
};

void RecyclerbotMaster::open_gripper(int whichArm)
{
  move_gripper(gripperOpen, whichArm);
}

void RecyclerbotMaster::close_gripper(double effort, int whichArm)
{
//	cout<<"++++++ closing effort: "<<effort<<endl;
//	cout<<"++++++ gripper_big_effort: "<<gripper_big_effort<<endl;
  move_gripper(gripperClose, effort, whichArm);
}

// move gripper to a position with a max effort
void RecyclerbotMaster::move_gripper(double position, double effort, int whichArm)
{
	GripperClient* gripperClient = (whichArm == RIGHT_ARM) ? gripperClientR : gripperClientL;
	
  gripperGoal.command.position = position;
  gripperGoal.command.max_effort = effort;
  
  ROS_INFO("Gripper action server sending goal.");
  gripperClient->sendGoal(gripperGoal);
  bool finished_before_timeout = gripperClient->waitForResult(ros::Duration(gripperTimeout));
  
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripperClient->getState();
    ROS_INFO("Gripper movement finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Gripper movement did not finish before the time out.");

  gripperPosition = gripperClient->getResult()->position;
    
}

void RecyclerbotMaster::move_arm_to_bin(geometry_msgs::PoseStamped& pose, int whichArm)
{
  geometry_msgs::PoseStamped initPose = (whichArm == RIGHT_ARM) ? initPoseR : initPoseL;
	
/*	double lowerDistance[3];
	lowerDistance[0] = 0.29;
	lowerDistance[1] = 0.62;
	lowerDistance[2] = 0.55;
*/	
	double liftDistance = 0.3;
	
  geometry_msgs::PoseStamped targetPose;
  
  // lift
  targetPose = currentPose;
  //targetPose.pose.position.z += liftDistance;
  targetPose.pose.position.z = tabletop + liftDistance;
  move_arm(targetPose, armPrecisionL, whichArm);
  ros::Duration(1).sleep();
  
  // plan a simple path
  geometry_msgs::Pose path2bin[3];
  
  geometry_msgs::Pose binR[3];
  geometry_msgs::Pose binL[3];
  
  int i = 0;
  for (i = 0; i < 3; i++)
  {
  	binR[i] = initPose.pose;
  	binL[i] = initPose.pose;
  }
  
  // 0 - can; 1 - glass bottle; 2 - plastic bottle
  binR[0].position.x = 0.262;
  binR[0].position.y = -0.145;
  binR[0].position.z = 1.018;
  binR[1].position.x = 0.201;
  binR[1].position.y = -0.575;
  binR[1].position.z = 0.648;
  binR[2].position.x = -0.086;
  binR[2].position.y = -0.551;
  binR[2].position.z = 0.647;
  
  binL[0].position.x = 0.254;
  binL[0].position.y = -0.174;
  binL[0].position.z = 1.0;
  binL[1].position.x = 0.617;
  binL[1].position.y = 0.586;
  binL[1].position.z = 0.851;
  binL[2].position.x = 0.242;
  binL[2].position.y = 0.599;
  binL[2].position.z = 0.936;
  
  geometry_msgs::Pose* bin = (whichArm == RIGHT_ARM) ? binR : binL;
  
	path2bin[0] = bin[category];
	path2bin[0].position.z = tabletop + liftDistance;
	if ((category == 2) && (whichArm == RIGHT_ARM)) path2bin[0].position.x += 0.4;
	path2bin[1] = bin[category];
	path2bin[2] = initPose.pose;

  targetPose = initPose;
  
  // move above bin
  targetPose.pose = path2bin[0];
  move_arm(targetPose, armPrecisionL, whichArm);
  ros::Duration(0.5).sleep();
  
  // move down
  targetPose.pose = path2bin[1];
  move_arm(targetPose, armPrecisionL, whichArm);
  ros::Duration(0.5).sleep();
  
  // open gripper
  open_gripper(whichArm);
  ros::Duration(0.5).sleep();
  
  // move up
  targetPose.pose = path2bin[2];
  move_arm(targetPose, armPrecisionL, whichArm);
  ros::Duration(0.5).sleep();
  
}


void RecyclerbotMaster::move_arm(geometry_msgs::PoseStamped& targetPose, double precision, int whichArm)
{
	GraspClient* graspClient = (whichArm == RIGHT_ARM) ? graspClientR : graspClientL;
	
  ROS_INFO("Arm action server sending goal.");
  
  // send a goal to the action
  goal.targetPose = targetPose;
  goal.precision = precision;
  graspClient->sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = graspClient->waitForResult(ros::Duration(armTimeout));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = graspClient->getState();
    currentPose = graspClient->getResult()->endPose;
    ROS_INFO("Arm movement finished: %s",state.toString().c_str());
  }
  else
  {
  	if (whichArm == RIGHT_ARM)
    	ROS_INFO("Right arm movement did not finish before the time out.");
    else
    	ROS_INFO("Left arm movement did not finish before the time out.");
  }
}

void RecyclerbotMaster::object_pose_callback(const recyclerbot::CylinderArray& msg)
{
  if ((state == DETECTION) && (msg.header.stamp > finishTime))
  {
    state = MOVE_CLOSE;

		int whichArm;
		n.param("using_arm", whichArm, RIGHT_ARM);
    open_gripper(whichArm);

    geometry_msgs::PoseStamped targetPose;
    targetPose.header.frame_id = msg.header.frame_id;
    targetPose.header.stamp = ros::Time::now();  
    targetPose.pose = msg.cylinders[0].pose;
    targetPose.pose.orientation = initPoseR.pose.orientation;
    tabletop = msg.cylinders[0].pose.position.z - msg.cylinders[0].height / 2;
    category = msg.cylinders[0].category;
    
    // move above the item
    targetPose.pose.position.y += (whichArm == RIGHT_ARM) ? 0.01 : -0.01;
    targetPose.pose.position.z = tabletop + 0.28;
    targetPose.pose.position.x -= 0.08;
    move_arm(targetPose, armPrecisionH, whichArm);
    
    use_low_speed(whichArm);
    ros::Duration(0.5).sleep();
    
    // move to the item
    targetPose.pose.position.z -= 0.15;
    targetPose.pose.position.x += 0.02;
    if (whichArm == RIGHT_ARM) targetPose.pose.position.y += 0.03;
    move_arm(targetPose, armPrecisionH, whichArm);
    ros::Duration(0.5).sleep();
    targetPose.pose.position.z = msg.cylinders[0].pose.position.z - 0.02;
    targetPose.pose.position.x += (0.08 - 0.01);
    if (whichArm == RIGHT_ARM) targetPose.pose.position.y -= 0.03;
    move_arm(targetPose, armPrecisionH, whichArm);
    ros::Duration(1).sleep();
    
    // squeeze
    
    if ((whichArm == LEFT_ARM) && false)
    {
		  //squeeze();
		  double step_effort = (max_squeeze_effort - min_squeeze_effort) / 10;
		  double effort = 0;
		  for (int sqzTime = 0; sqzTime <= 10; sqzTime++)
		  {
		  	effort = min_squeeze_effort + step_effort * sqzTime;
				close_gripper(effort, whichArm);
				cout<<"!!!!!!!!!!!!!!! effort = "<<effort<<endl;
				ros::Duration(1).sleep();
		  }
		  
		  
		  open_gripper(whichArm);
		  ros::Duration(1).sleep();
		  targetPose.pose.position.x -= 0.35;
		  targetPose.pose.position.z += 0.08;
		  move_arm(targetPose, armPrecisionL, whichArm);
		  ros::Duration(5).sleep();
		//  targetPose = initPoseL;
		//  move_arm(targetPose, armPrecisionL, whichArm);
		  
			state = DETECTION;
			finishTime = ros::Time::now();
    
		  return;
    }
    
    use_high_speed(whichArm);
    ros::Duration(0.5).sleep();
    
    // grasp
    if (category == 1) close_gripper(gripper_big_effort, whichArm);
    else close_gripper(gripper_small_effort, whichArm);
    
    // distinguish between glass/plastic bottle
    if (category > 0) // not a can
    {
    	if (gripperPosition > 0.068) // this is a glass bottle
    	{
    		if (category != 1)
    		{
    			close_gripper(gripper_big_effort, whichArm);
    			ros::Duration(1).sleep();
    			if (gripperPosition > 0.068) category = 1;
    		}
    	}
    	else category = 2;
    }
    
    if (category == 0) close_gripper(gripper_small_effort-5, whichArm);
    if (category == 2) close_gripper(gripper_small_effort, whichArm);
    
    // be careful with glass bottles
    if (category == 1) use_medium_speed(whichArm);
    ros::Duration(0.5).sleep();
    
    // move to bin
    move_arm_to_bin(targetPose, whichArm);
    
    if (category == 1) use_high_speed(whichArm);
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
