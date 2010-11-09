#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"
//#include "rosbag/bag.h"
#include <tf/transform_listener.h>
#include "recyclerbot/table_detector.h"
#include "recyclerbot/STANN/rand.hpp"

using namespace std;

class TableDetectorNode
{
public:
  ros::NodeHandle n;
	ros::Subscriber ntPoint_sub; // nt = narrow textured point cloud
	ros::Subscriber wsPoint_sub; // ws = wide stereo point cloud
	ros::Publisher filteredPoints_pub; // publish processed points
	ros::Publisher cylMarker_pub; // publish cylinders
	tf::TransformListener* tran; // narrow textured to Footprint
	ros::Time preStamp;
	sensor_msgs::PointCloud wsPoints;
	TableDetector tableDetector;
	
	std_msgs::ColorRGBA colorArray[COLORNUM];
  
  TableDetectorNode(ros::NodeHandle &_n) : n(_n)
  {
    filteredPoints_pub = n.advertise<sensor_msgs::PointCloud>("/filtered_points", 10);
    cylMarker_pub = n.advertise<visualization_msgs::Marker>("/bottle_marker", 10);
    ntPoint_sub = n.subscribe<sensor_msgs::PointCloud>("/narrow_stereo_textured/points", 1, &TableDetectorNode::nt_cb, this);
    wsPoint_sub = n.subscribe<sensor_msgs::PointCloud>("/wide_stereo/points", 1, &TableDetectorNode::ws_cb, this);
    
    tran = new tf::TransformListener(n, ros::Duration(2.0));
    preStamp = ros::Time::now();
//    ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&TableDetectorNode::transformPoint, boost::ref(listener)));

		// initialize color array
		for (int i = 0; i < COLORNUM; i++)
		{
			colorArray[i].r = color_array[i][0];
			colorArray[i].g = color_array[i][1];
			colorArray[i].b = color_array[i][2];
			colorArray[i].a = color_array[i][3];
		}
  }
  
  ~TableDetectorNode()
  {
    ROS_INFO("TableDetectorNode destructor");
    delete tran;
  }
  
  void nt_cb(const sensor_msgs::PointCloud::ConstPtr &msg); // narrow textured call back
  void ws_cb(const sensor_msgs::PointCloud::ConstPtr &msg); // wide stereo call back
};


void TableDetectorNode::ws_cb(const sensor_msgs::PointCloud::ConstPtr &msg) // wide stereo call back
{
	wsPoints = *msg;
}
  
void TableDetectorNode::nt_cb(const sensor_msgs::PointCloud::ConstPtr &msg) // narrow textured call back
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
	tableDetector.process_cloud(ntTrPoints, wsTrPoints, cylNum, filtered_msgs, marker_msgs);
	
	if (n.ok()) 
	{
		for (i = 0; i < filtered_msgs.size(); i++) 
		{
			// copy header info into new message
			filtered_msgs[i].header.frame_id = "/base_footprint";
			filtered_msgs[i].header.stamp = ros::Time::now();
			filtered_msgs[i].header.seq = msg->header.seq;
			filteredPoints_pub.publish(filtered_msgs[i]);
		}
		for (i = 0; i < marker_msgs.size(); i++) cylMarker_pub.publish(marker_msgs[i]);
	}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_detector");
  ros::NodeHandle n;
  __srand48__(static_cast<unsigned int>(time(0)));
  TableDetectorNode tdn(n);
	ros::spin();
  return 0;
}


