/*
 * Project 3
 * Team Win: Arne Bech, Brian Chung, Robert Kanter, Pierre Kreitmann
 */

#include "Publisher.h"
#include "util.h"

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
	mark.ns = "Pose_distribution";
	mark.action = visualization_msgs::Marker::ADD;
	mark.pose.orientation.w = 1.0;
	mark.id = 0;
	mark.type = visualization_msgs::Marker::POINTS;
	mark.color.r = 1.0;
	mark.color.b = 0.5;
	mark.color.a = 1.0;
	mark.scale.x = 0.02;
	mark.scale.y = 0.02;
	for(unsigned int i = 0; i < poses.size(); i = i + 1)
	{
		geometry_msgs::Point point;
	    point.x = poses.at(i)->getX();
	    point.y = poses.at(i)->getY();
	    point.z = 0.0;
	    
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
 * Publishes a box approximating the robot's location to rviz
 */
void Publisher::publishRobot(ros::Publisher marker_pub)
{
    visualization_msgs::Marker window;
    
    window.header.frame_id = *Util::baseFootprintFrameId;
	window.header.stamp = ros::Time::now();
	window.ns = "Robot";
	window.action = visualization_msgs::Marker::ADD;
	window.pose.orientation.w = 1.0;
	window.id = 1;
	window.type = visualization_msgs::Marker::LINE_STRIP;
	window.color.a = 1.0;
	window.scale.x = 0.05;
	window.color.b = 1.0;
	window.scale.y = 0.01;

    
     geometry_msgs::Point point;
     point.x = 0.2;
     point.y = 0.2;
     point.z = 0.0;
     window.points.push_back(point);
     
     point.x = 0.2;
     point.y = -0.2;
     point.z = 0.0;
     window.points.push_back(point);
     
     point.x = -0.2;
     point.y = -0.2;
     point.z = 0.0;
     window.points.push_back(point);
     
     point.x = -0.2;
     point.y = 0.2;
     point.z = 0.0;
     window.points.push_back(point);
     
     point.x = 0.2;
     point.y = 0.2;
     point.z = 0.0;
     window.points.push_back(point);
    
     marker_pub.publish(window);
    
}

/*
 * Helper used by publishMap to determine the colors
 */
std_msgs::ColorRGBA Publisher::colorScale(double x) 
{
  std_msgs::ColorRGBA color;
  color.a = .5;
  x = 2*x/3;
  color.r = max(0.0, min(1.0, max(2.0 - 6.0*x, 6.0*x - 4.0)));
  color.g = max(0.0, min(1.0, min(4.0 - 6.0*x, 6.0*x)));
  color.b = max(0.0, min(1.0, min(6.0 - 6.0*x, 6.0*x - 2.0)));

  return color;
}

void Publisher::publishMap(const Map & map, const ros::Publisher & publisher, const char* nameSpace, std::string frameID="/map")
{
    static double lastPublishedTime = -1;
    double currentTime =ros::Time::now().toSec();
    
    if ((currentTime - lastPublishedTime) < 10)  {
        return;
    }
   
   lastPublishedTime = currentTime;
  /* variables starting with m are in meters, those starting with px are in
   * pixels
   */
  float mMaxValue = 20.0; // all values above (ie., obstacles, that are INF) are not published
    float mMinValue = 0.0;
    
    if (frameID == "/odom") {
        mMaxValue = 1.1;
        mMinValue = 0.8;
    }
  visualization_msgs::Marker marker;
  marker.header.frame_id = frameID;
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration::Duration();
  marker.ns = nameSpace;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = map.resolution();
  marker.scale.y = marker.scale.x;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.color.a= 1.0;

  int cols = map.cols();
  int rows = map.rows();

  for (int pxY = 0; pxY < rows; pxY++) {
    for (int pxX = 0; pxX < cols; pxX++) {
      double mVal = map(pxX, pxY);

      if (mMinValue<= mVal && mVal <= mMaxValue ) {
        double tempVal = (mVal-mMinValue)/(mMaxValue-mMinValue);
        std_msgs::ColorRGBA myColor = colorScale(tempVal);
        marker.colors.push_back(myColor);

        geometry_msgs::Point myPoint;
        myPoint.x = (pxX+ .5) * map.resolution();
        myPoint.y = (pxY+ .5) * map.resolution();
        // the .5 is there because we give the center of the square
        myPoint.z = 0;
        marker.points.push_back(myPoint);
      }
    }
  }
  publisher.publish(marker);
}
