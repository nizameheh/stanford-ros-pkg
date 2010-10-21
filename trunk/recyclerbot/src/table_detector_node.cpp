#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "geometry_msgs/Point32.h"
#include "rosbag/bag.h"
#include "recyclerbot/bottles_and_can_data.h"
#include <tf/transform_listener.h>

#define DATAPATH "/home/jiahui/test_data/bottles_and_can_data.bag"

using namespace std;

class tableDetectorNode
{
public:
  ros::NodeHandle n;
  
  tableDetectorNode()//(ros::NodeHandle &_n) : n(_n), it(_n)
  {
    //image_sub = it.subscribe("image", 1, &tableDetectorNode::image_cb, this);
  }
  ~tableDetectorNode()
  {
    ROS_INFO("tableDetectorNode destructor");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_detector");
  ros::NodeHandle n;
  
  int i = 0;
  double maxZ = 0;
  double minZ = 100;
  // find max and min z value
  for (i = 0; i < pointCloudNum; i++) 
  {
  	if (pointCloud[i][2] > maxZ) maxZ = pointCloud[i][2];
  	if (pointCloud[i][2] < minZ) minZ = pointCloud[i][2];
  }
  cout << maxZ << "; " << minZ << endl;
  // count along z axis
	int countAlongZ[14];
	for (i=0;i<14;i++) countAlongZ[i]=0;
	  
	for (i = 0; i < pointCloudNum; i++) 
  {
		countAlongZ[int(pointCloud[i][2]*10)]++;
	}
	for (i=0;i<14;i++) cout << countAlongZ[i] << endl;

	ros::Publisher filtered_points_pub = n.advertise<sensor_msgs::PointCloud>("filtered_points", 1000);
	
	// display new point cloud
	int newPointNum = pointCloudNum-countAlongZ[6];
	sensor_msgs::PointCloud filtered_msg;
	filtered_msg.header.frame_id = "/wide_stereo_optical_frame";
	geometry_msgs::Point32 p_;
	sensor_msgs::ChannelFloat32 ch_;
	int j=0;
	for (i=0; i<pointCloudNum; i++)
	{
		if ((pointCloud[i][2]<0.6)||(pointCloud[i][2]>=0.7)) {
			p_.x = pointCloud[i][0];
			p_.y = pointCloud[i][1];
			p_.z = pointCloud[i][2];
			ch_.values.push_back(0.5);
			filtered_msg.points.push_back(p_);
			j++;
			
			if (j >= newPointNum) 
			{
				cout << "j>=newPointNum" << endl;
				break;
			}
		}
	}
	ch_.name = "r";
	filtered_msg.channels.push_back(ch_);
	ch_.name = "g";
	filtered_msg.channels.push_back(ch_);
	ch_.name = "b";
	filtered_msg.channels.push_back(ch_);
	
	ros::Rate rate(10);
	tf::TransformListener lr;
	tf::StampedTransform transform;
	while (n.ok()) {
		lr.lookupTransform("/base_footprint","/base_footprint",ros::Time(0),transform);
		filtered_msg.header.stamp = transform.stamp_;
		filtered_points_pub.publish(filtered_msg);
		rate.sleep();
	}
	
	ros::spinOnce();
/*
  tableDetectorNode tdn(n);
  ros::spin();
  */
  
  return 0;
}
