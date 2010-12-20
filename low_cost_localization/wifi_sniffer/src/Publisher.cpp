

#include "Publisher.h"

/*
 * Publishes a single Pose to rviz
 */
void Publisher::publishPoint(ros::Publisher marker_pub, Pose* pose, std::string frameID, std::string nameS, int id = 0)
{
	visualization_msgs::Marker mark;
	mark.header.frame_id = frameID;
	mark.header.stamp = ros::Time::now();
	mark.ns = nameS;
	mark.action = visualization_msgs::Marker::ADD;
	mark.pose.orientation.w = 1.0;
	mark.id = id;
	mark.type = visualization_msgs::Marker::CUBE;
	mark.color.r = 0.8;
	mark.color.b = 0.9;
	mark.color.a = 0.3;
	mark.scale.x = 0.3;
	mark.scale.y = 0.3;
	mark.scale.z = 0.3;
	
	mark.pose.position.x = pose->getX();
	mark.pose.position.y = pose->getY();
	//ROS_INFO("Best pose: (%f, %f)\n", pose->getX(), pose->getY());
	mark.pose.position.z = 0.0;
	mark.pose.orientation.w = 1;
	
	
    marker_pub.publish(mark);
}

void Publisher::publishPoint2(ros::Publisher marker_pub, Pose* pose, std::string frameID, std::string nameS, int id = 0)
{
	visualization_msgs::Marker mark;
	mark.header.frame_id = frameID;
	mark.header.stamp = ros::Time::now();
	mark.ns = nameS;
	mark.action = visualization_msgs::Marker::ADD;
	mark.pose.orientation.w = 1.0;
	mark.id = id;
	mark.type = visualization_msgs::Marker::ARROW;
	mark.color.r = 1.0;
	mark.color.b = 1.0;
	mark.color.g = 1.0;
	mark.color.a = 0.9;
	mark.scale.x = 0.1;
	mark.scale.y = 0.2;
	mark.scale.z = 0.1;
	
	mark.pose.position.x = pose->getX();
	mark.pose.position.y = pose->getY();
	//ROS_INFO("Best pose: (%f, %f)\n", pose->getX(), pose->getY());
	mark.pose.position.z = 0.01;
	//mark.pose.orientation.w = 1;
	mark.pose.orientation = tf::createQuaternionMsgFromYaw(pose->getTheta());
	
	
    marker_pub.publish(mark);
}


/*
 * Publishes a PDF as points to rviz
 */
void Publisher::publishPDF(ros::Publisher marker_pub, std::vector<Pose*> poses, std::string frameID="/map")
{
	visualization_msgs::Marker mark;
	mark.header.frame_id = frameID;
	mark.header.stamp = ros::Time::now();
	mark.ns = "poses";
	mark.action = visualization_msgs::Marker::ADD;
	mark.pose.orientation.w = 1.0;
	mark.pose.orientation.x = 0.0;
	mark.pose.orientation.y = 0.0;
	mark.pose.orientation.z = 0.0;
	mark.id = 0;
	mark.type = visualization_msgs::Marker::POINTS;
	//mark.color.r = 1.0;
	//mark.color.b = 0.5;
	mark.color.a = 1.0;
	mark.scale.x = 0.02;
	mark.scale.y = 0.02;
	
	float maxProb = 0.0;
	for(unsigned int a = 0; a < poses.size(); a++) {
		if(poses.at(a)->getProbability() > maxProb) maxProb = poses.at(a)->getProbability();
	}
	
	for(unsigned int i = 0; i < poses.size(); i = i + 1)
	{
		geometry_msgs::Point point;
		float colorNumber = poses.at(i)->getProbability() / maxProb;
		colorNumber = std::max(colorNumber, 0.0f);
		colorNumber = std::min(colorNumber, 1.0f);
		std_msgs::ColorRGBA myColor = colorScale(colorNumber);
		//ROS_WARN("r: %f, g: %f, b: %f, a: %f", myColor.r, myColor.g, myColor.b, myColor.a);
		mark.colors.push_back(myColor);
	    point.x = poses.at(i)->getX();
	    point.y = poses.at(i)->getY();
	    point.z = 0.1;
	    
	    //ROS_INFO("x: %f, y: %f", point.x, point.y);
	    
    	mark.points.push_back(point);
    }
    marker_pub.publish(mark);
}

/*
 * Publishes a PDF as arrows to rviz
 */
void Publisher::publishArrowsPDF(ros::Publisher posePublisher, std::vector<Pose*> poses, std::string frameID="/map", float offset = 0.0) {
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = frameID;
	msg.header.stamp = ros::Time::now();
	
	for(unsigned int i = 0; i < poses.size(); i++)
	{
	   geometry_msgs::Pose pose;
	    pose.position.x = poses.at(i)->getX() - offset;
	    pose.position.y = poses.at(i)->getY() - offset;
	    pose.position.z = 0.0;
	    
	    pose.orientation = tf::createQuaternionMsgFromYaw(poses.at(i)->getTheta());
	    
	    //ROS_INFO("x: %f, y: %f", point.x, point.y);
	    
    	msg.poses.push_back(pose);
    }
    posePublisher.publish(msg); 
}

/*
 * Helper used by publishMap to determine the colors
 */
std_msgs::ColorRGBA Publisher::colorScale(double x) 
{
  std_msgs::ColorRGBA color;
  color.a = .5;
  //x = 2*x/3;
  color.r = x;
  color.g = 1.0 - x;
  color.b = 1.0 - x;
  
  //color.r = std::max(0.0, std::min(1.0, std::max(2.0 - 6.0*x, 6.0*x - 4.0)));
  //color.g = 0.0; //std::max(0.0, std::min(1.0, std::min(4.0 - 6.0*x, 6.0*x)));
  //color.b = 1.0 - color.r; //std::max(0.0, std::min(1.0, std::min(6.0 - 6.0*x, 6.0*x - 2.0)));

  return color;
}
