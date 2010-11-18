#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <recyclerbot/GraspAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
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
  ros::Subscriber currentPose_sub;
  
  bool success;
  double positionError;

public:
  
  GraspServer(std::string name)
  {
    server = new Server(n, name, boost::bind(&GraspServer::execute_cb, this, _1));
    actionName = name;
    
    n.getParam("success_threshold", success_threshold);
    n.getParam("max_lead", max_lead);
    
    targetPose_pub = n.advertise<geometry_msgs::PoseStamped>("target_pose", 3);
    commandPose_pub = n.advertise<geometry_msgs::PoseStamped>("command_pose", 3);
    
    currentPose_sub = n.subscribe("current_pose", 1, &GraspServer::cur_pose_cb, this);
    targetSet = false;
    positionError = 0;
    
    ROS_INFO_STREAM("success_threshold: " << success_threshold);
  }
  
  void cur_pose_cb(const geometry_msgs::PoseStamped& msg)
  {
    geometry_msgs::Pose curPose = msg.pose;
    if (targetSet == true)
    {
      positionError = euclidean_distance(targetPose.pose.position, currentPose.pose.position);
    }
  }
  
  void execute_cb(const recyclerbot::GraspGoalConstPtr &goal)
  {
   success = true;
   
   ros::Rate rate(10.0);
   targetPose = goal->targetPose;
   targetSet = true;
   commandPose_pub.publish(goal->targetPose);
   
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
    /*
    if (positionError > max_lead)
    {
      double k = max_lead / positionError;
      wayPoint.pose.position.x = targetPose.pose.position.x * k + currentPose.pose.position.x * (1 - k);
      wayPoint.pose.position.y = targetPose.pose.position.y * k + currentPose.pose.position.y * (1 - k);
      wayPoint.pose.position.z = targetPose.pose.position.z * k + currentPose.pose.position.z * (1 - k);
    
    }*/
    commandPose_pub.publish(wayPoint);
    feedback.currentPose = currentPose.pose;
    rate.sleep();    
   }
   if (success == true)
   {
    result.endPose = currentPose.pose;
    server->setSucceeded(result);
   }
   
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
