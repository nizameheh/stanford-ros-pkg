#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"
//#include "rosbag/bag.h"
#include <tf/transform_listener.h>
#include "recyclerbot/table_detector.h"

using namespace std;

class TableDetectorNode
{
public:
  ros::NodeHandle n;
	ros::Subscriber ntPoint_sub; // nt = narrow textured point cloud
	ros::Publisher filteredPoints_pub; // publish processed points
	tf::TransformListener* tran; // narrow textured to Footprint
	ros::Time preStamp;
	TableDetector tableDetector;
  
  TableDetectorNode(ros::NodeHandle &_n) : n(_n)
  {
    filteredPoints_pub = n.advertise<sensor_msgs::PointCloud>("/filtered_points", 1);
    ntPoint_sub = n.subscribe<sensor_msgs::PointCloud>("/narrow_stereo_textured/points", 1, &TableDetectorNode::nt_cb, this);
    tran = new tf::TransformListener(n, ros::Duration(2.0));
    preStamp = ros::Time::now();
//    ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&TableDetectorNode::transformPoint, boost::ref(listener)));

  }
  
  ~TableDetectorNode()
  {
    ROS_INFO("TableDetectorNode destructor");
    delete tran;
  }
  
  void nt_cb(const sensor_msgs::PointCloud::ConstPtr &msg) // narrow textured call back
  {
  	// tranform point cloud into /base_footprint frame
  	sensor_msgs::PointCloud transformedPoints;
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
  		tran->transformPointCloud("/base_footprint", *msg, transformedPoints);
    } 
    catch (tf::TransformException& ex) 
    {
    	return;
    }
    
  	sensor_msgs::PointCloud filtered_msg;
  	// copy header info into new message
		filtered_msg.header.frame_id = "/base_footprint";//msg->header.frame_id;
		filtered_msg.header.stamp = ros::Time::now();
		filtered_msg.header.seq = msg->header.seq;
		// get table point cloud
  	tableDetector.process_cloud(transformedPoints, &filtered_msg);
  	//cout<<filtered_msg.header.seq<<endl;
  	//cout<<"size: "<<filtered_msg.points.size()<<endl;
		if (n.ok()) filteredPoints_pub.publish(filtered_msg);
//		ros::spinOnce();
/*    geometry_msgs::PointStamped m;
  	for (int i = 0; i < msg->points.length(); i++) 
  	{
  		tran->transformPoint("/base_footprint", msg->points(i), m);
  		transformedPoints.push_back();
  	}*/
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_detector");
  ros::NodeHandle n;
  TableDetectorNode tdn(n);
	ros::spin();
  return 0;
}
