#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <recyclerbot/GraspAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <math.h>

using namespace std;

typedef actionlib::SimpleActionServer<recyclerbot::GraspAction> Server;

class GraspServer
{
private:
  
  ros::NodeHandle n;
  Server* server;
  std::string actionName;
  recyclerbot::GraspFeedback feedback;
  recyclerbot::GraspResult result;
  
  double success_threshold;
  double max_lead;
  
  geometry_msgs::PoseStamped targetPose;
  geometry_msgs::PoseStamped currentPose;
  bool targetSet;
  
  ros::Publisher targetPose_pub;
  ros::Publisher commandPose_pub;
  //ros::Publisher velocitySaturation_pub;
  ros::Subscriber currentPose_sub;
  tf::TransformListener* trans;
  
  bool success;
  double positionError;
  string curPoseFrameId;

public:
  
  GraspServer(std::string name)
  {
    trans = new tf::TransformListener(n, ros::Duration(3.0));
    server = new Server(n, name, boost::bind(&GraspServer::execute_cb, this, _1));
    actionName = name;
    
    n.getParam("success_threshold", success_threshold);
    n.getParam("max_lead", max_lead);
    
    targetPose_pub = n.advertise<geometry_msgs::PoseStamped>("target_pose", 3);
    commandPose_pub = n.advertise<geometry_msgs::PoseStamped>("command_pose", 3);
    //velocitySaturation_pub = n.advertise<geometry_msgs::PoseStamped>("velocity_saturation", 3);
    
    currentPose_sub = n.subscribe("current_pose", 1, &GraspServer::cur_pose_cb, this);
    targetSet = false;
    positionError = 0;
    curPoseFrameId = "";
    
    ROS_INFO_STREAM("success_threshold: " << success_threshold);
  }
  
  void cur_pose_cb(const geometry_msgs::PoseStamped& msg)
  {
    currentPose = msg;
    if (curPoseFrameId == "") curPoseFrameId = msg.header.frame_id;
    
    if (targetSet == true)
    {
      positionError = euclidean_distance(targetPose.pose.position, currentPose.pose.position);
     // cout<<"positionError: "<<positionError<<endl;
    }
  }
  
  void execute_cb(const recyclerbot::GraspGoalConstPtr &goal)
  {
  	success_threshold = goal->precision;
    ros::Rate rate(10.0);
    while (curPoseFrameId == "") rate.sleep();
    
    geometry_msgs::PoseStamped tempPose;
    tempPose = goal->targetPose;
    tempPose.header.stamp = ros::Time(0);
    try 
    {
      trans->transformPose(curPoseFrameId, tempPose, targetPose);
    }
    catch (tf::TransformException& ex) 
    {
      cout<<"Transform fail..... "<<ex.what()<<endl;
      return;
    }
    
    targetSet = true;
    commandPose_pub.publish(targetPose);
    success = true;
   
    positionError = 0;
    while (positionError == 0) rate.sleep();
    while ((positionError > success_threshold) && n.ok())
    {
      if (server->isPreemptRequested())
      {
	server->setPreempted();
	success = false;
	break;
      }
      geometry_msgs::PoseStamped wayPoint;
      wayPoint = targetPose;
      wayPoint.header.stamp = ros::Time::now();
      
      if (positionError > max_lead)
      {
	double k = max_lead / positionError;
	wayPoint.pose.position.x = targetPose.pose.position.x * k + currentPose.pose.position.x * (1 - k);
	wayPoint.pose.position.y = targetPose.pose.position.y * k + currentPose.pose.position.y * (1 - k);
	wayPoint.pose.position.z = targetPose.pose.position.z * k + currentPose.pose.position.z * (1 - k);
      
      }
      commandPose_pub.publish(wayPoint);
      feedback.currentPose = currentPose;
      rate.sleep();    
    }
    if (success == true)
    {
      result.endPose = currentPose;
      server->setSucceeded(result);
    }
    targetSet = false;
  }
  
  // calculate euclidean distance of two points
  double euclidean_distance(geometry_msgs::Point& p1, geometry_msgs::Point& p2)
  {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_server");  
  GraspServer graspServer(ros::this_node::getName());
  ros::spin();

  return 0;
}
