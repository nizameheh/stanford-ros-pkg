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

enum State {DETECTION, MOVECLOSE, GRASP, MOVEAWAY, DROP};
typedef actionlib::SimpleActionClient<recyclerbot::GraspAction> GraspClient;

class RecyclerbotMaster
{
  private:
    
  ros::NodeHandle n;
  ros::Subscriber cylinderArray_pub;
  GraspClient* client;
  
  State state;
  
  public:
    
  RecyclerbotMaster(ros::NodeHandle &_n) : n(_n)
  {
    state = DETECTION;
    cylinderArray_pub = n.subscribe("/cylinder_array", 10, &RecyclerbotMaster::object_pose_callback, this);
    
    client = new GraspClient("grasp_server", true);
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    client->waitForServer(); //will wait for infinite time
    
    ROS_INFO("Action server started");
  }
  void object_pose_callback(const recyclerbot::CylinderArray& msg);
    
};

void RecyclerbotMaster::object_pose_callback(const recyclerbot::CylinderArray& msg)
{
  if (state == DETECTION)
  {
    state = MOVECLOSE;
    ROS_INFO("Action server sending goal.");
    
    // send a goal to the action
    geometry_msgs::PoseStamped targetPose;
    targetPose.header.frame_id = msg.header.frame_id;
    targetPose.header.stamp = ros::Time::now();  
    targetPose.pose = msg.cylinders[0].pose;
    
    recyclerbot::GraspGoal goal;
    goal.targetPose = targetPose;
    client->sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = client->waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = client->getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
    
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
