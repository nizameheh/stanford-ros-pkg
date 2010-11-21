#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
//#include "rosbag/bag.h"
#include <tf/transform_listener.h>
#include <recyclerbot/object_detector.h>
#include <recyclerbot/STANN/rand.hpp>
#include <recyclerbot/CylinderArray.h>

using namespace std;

class ObjectDetectorNode
{
  private:
  ros::NodeHandle n;
  ros::Subscriber ntPoint_sub; // nt = narrow textured point cloud
  ros::Subscriber wsPoint_sub; // ws = wide stereo point cloud
  ros::Publisher filteredPoints_pub; // publish processed points
  ros::Publisher objectPoints_pub; // publish processed points
  ros::Publisher cylMarker_pub; // publish cylinders
  ros::Publisher cylinderArray_pub;
  tf::TransformListener* tran; // narrow textured to Footprint
  ros::Time preStamp;
  sensor_msgs::PointCloud wsPoints;
  ObjectDetector objectDetector;
  
  std_msgs::ColorRGBA colorArray[COLORNUM];
  
  public:
  ObjectDetectorNode(ros::NodeHandle &_n) : n(_n)
  {
    filteredPoints_pub = n.advertise<sensor_msgs::PointCloud>("/filtered_points", 10);
    objectPoints_pub = n.advertise<sensor_msgs::PointCloud>("/object_points", 10);
    cylMarker_pub = n.advertise<visualization_msgs::Marker>("/bottle_marker", 10);
    cylinderArray_pub = n.advertise<recyclerbot::CylinderArray>("/cylinder_array", 10);
    ntPoint_sub = n.subscribe<sensor_msgs::PointCloud>("/narrow_stereo_textured/points", 1, &ObjectDetectorNode::nt_cb, this);
    wsPoint_sub = n.subscribe<sensor_msgs::PointCloud>("/wide_stereo/points", 1, &ObjectDetectorNode::ws_cb, this);
    
    tran = new tf::TransformListener(n, ros::Duration(2.0));
    preStamp = ros::Time::now();
//  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&ObjectDetectorNode::transformPoint, boost::ref(listener)));

    // initialize color array
    for (int i = 0; i < COLORNUM; i++)
    {
      colorArray[i].r = color_array[i][0];
      colorArray[i].g = color_array[i][1];
      colorArray[i].b = color_array[i][2];
      colorArray[i].a = color_array[i][3];
    }
  }
  
  ~ObjectDetectorNode()
  {
    ROS_INFO("ObjectDetectorNode destructor");
    delete tran;
  }
  
  void nt_cb(const sensor_msgs::PointCloud::ConstPtr &msg); // narrow textured call back
  void ws_cb(const sensor_msgs::PointCloud::ConstPtr &msg); // wide stereo call back
};


void ObjectDetectorNode::ws_cb(const sensor_msgs::PointCloud::ConstPtr &msg) // wide stereo call back
{
  wsPoints = *msg;
}


void ObjectDetectorNode::nt_cb(const sensor_msgs::PointCloud::ConstPtr &msg) // narrow textured call back
{
  if (wsPoints.points.empty()) return;
  
  int cylNum = 0;
  // tranform point cloud into /base_footprint frame
  sensor_msgs::PointCloud ntTrPoints; // narrow textured transformed points
  sensor_msgs::PointCloud wsTrPoints;
  // deal with rosbag flushing
  if (preStamp >= msg->header.stamp)
  {
    cout<<"flush listener"<<endl;
    preStamp = msg->header.stamp;
    tran->clear();
    return;
  }
  preStamp = msg->header.stamp;
  try 
  {
    tran->transformPointCloud("/base_footprint", *msg, ntTrPoints);
    tran->transformPointCloud("/base_footprint", wsPoints, wsTrPoints);
  }
  catch (tf::TransformException& ex) 
  {
    return;
  }
  
  int i = 0;
  
  vector<sensor_msgs::PointCloud> filtered_msgs;
  vector<visualization_msgs::Marker> marker_msgs;
  
  // get table point cloud
  objectDetector.process_cloud(ntTrPoints, wsTrPoints, cylNum, filtered_msgs, marker_msgs);
  
  
  if (marker_msgs.empty() == false)
  {
		// publish cylinders
		recyclerbot::CylinderArray cylinderArray;
		cylinderArray.header.frame_id = "/base_footprint";
		cylinderArray.header.stamp = msg->header.stamp;
		
		recyclerbot::Cylinder tempCylinder;
		tempCylinder.pose = marker_msgs[0].pose;
		
		n.getParam("object_pose/orientation/x", tempCylinder.pose.orientation.x);
		n.getParam("object_pose/orientation/y", tempCylinder.pose.orientation.y);
		n.getParam("object_pose/orientation/z", tempCylinder.pose.orientation.z);
		n.getParam("object_pose/orientation/w", tempCylinder.pose.orientation.w);
		
		tempCylinder.radius = marker_msgs[0].scale.x / 2;
		tempCylinder.height = marker_msgs[0].scale.z;
		
		if (marker_msgs[0].color.r == 1) tempCylinder.category = 1;
		else tempCylinder.category = 2;
		
		cylinderArray.cylinders.push_back(tempCylinder);
		cylinderArray_pub.publish(cylinderArray);
  }
  
  
  
  
  
  
  
  // publish markers and point cloud messages
  if (n.ok()) 
  {
//    for (i = 0; i < int(filtered_msgs.size()); i++)
		if (filtered_msgs.size() > 0)
    {
      // copy header info into new message
      filtered_msgs[0].header.frame_id = "/base_footprint";
      filtered_msgs[0].header.stamp = ros::Time::now();
      filtered_msgs[0].header.seq = msg->header.seq;
      objectPoints_pub.publish(filtered_msgs[0]);
    }
		if (filtered_msgs.size() > 1)
    {
      // copy header info into new message
      filtered_msgs[1].header.frame_id = "/base_footprint";
      filtered_msgs[1].header.stamp = ros::Time::now();
      filtered_msgs[1].header.seq = msg->header.seq;
      filteredPoints_pub.publish(filtered_msgs[1]);
    }
    
    for (i = 0; i < int(marker_msgs.size()); i++) cylMarker_pub.publish(marker_msgs[i]);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_detector");
  ros::NodeHandle n;
  __srand48__(static_cast<unsigned int>(time(0)));
  
  
  
  bool testing = false;
  if (testing)
  {
		// FOR TESTING ONLY!!!!
		ros::Rate rate(0.5);
		ros::Publisher cylinderArray_pub = n.advertise<recyclerbot::CylinderArray>("/cylinder_array", 10);

		recyclerbot::Cylinder tempCylinder;
		n.getParam("object_pose/position/x", tempCylinder.pose.position.x);
		n.getParam("object_pose/position/y", tempCylinder.pose.position.y);
		n.getParam("object_pose/position/z", tempCylinder.pose.position.z);
		n.getParam("object_pose/orientation/x", tempCylinder.pose.orientation.x);
		n.getParam("object_pose/orientation/y", tempCylinder.pose.orientation.y);
		n.getParam("object_pose/orientation/z", tempCylinder.pose.orientation.z);
		n.getParam("object_pose/orientation/w", tempCylinder.pose.orientation.w);
		tempCylinder.radius = 0.032914805061;
		tempCylinder.height = 0.143234491348;
		  
		while (n.ok()) 
		{
		    // publish cylinders
		  recyclerbot::CylinderArray cylinderArray;
		  cylinderArray.header.frame_id = "/base_footprint";
		  cylinderArray.header.stamp = ros::Time::now();
		  cylinderArray.cylinders.push_back(tempCylinder);
		  
		  cylinderArray_pub.publish(cylinderArray);
		  
		  rate.sleep();
		}
  }
  
  ObjectDetectorNode objectDetectorNode(n);
  
  ros::spin();
  return 0;
}


