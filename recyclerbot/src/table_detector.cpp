#include "recyclerbot/table_detector.h"

using namespace std;

TableDetector::TableDetector()
{

}

/*
PointCloud
Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point32[] points
  float32 x
  float32 y
  float32 z
sensor_msgs/ChannelFloat32[] channels
  string name
  float32[] values
*/
 
void TableDetector::process_cloud(
sensor_msgs::PointCloud& original_msg,
sensor_msgs::PointCloud* filtered_msg
)
{
	vector<geometry_msgs::Point32>& pointCloud = original_msg.points;
  int i = 0;
	int j = 0;  
  double maxZ = 0;
  double minZ = 100;
  int pointCloudNum = pointCloud.size();
  // find max and min z value
  for (i = 0; i < pointCloudNum; i++) 
  {
  	if (pointCloud[i].z > maxZ) maxZ = pointCloud[i].z;
  	if (pointCloud[i].z < minZ) minZ = pointCloud[i].z;
  }
  //cout << maxZ << "; " << minZ << endl;
  // count along z axis
	int countAlongZ[50];
	for (i = 0; i < 50; i++) countAlongZ[i]=0;
	  
	for (i = 0; i < pointCloudNum; i++) 
  {
		countAlongZ[int(pointCloud[i].z*100/2)]++;
	}
	//for (i = 0; i < 50; i++) cout<<i<<": " << countAlongZ[i] << endl;
	int maxSlot = 0;
	for (i = 0; i < 50; i++) 
	{
		if (countAlongZ[i] > countAlongZ[maxSlot])
		{
			maxSlot = i;
		}
	}
	//cout<<"maxSlot: "<<maxSlot<<endl;
	// find filtered point cloud
	int newPointNum = countAlongZ[maxSlot];
	geometry_msgs::Point32 p_;
	sensor_msgs::ChannelFloat32 ch0_, ch1_, ch2_;
	ch0_.name = original_msg.channels[0].name;
	ch1_.name = original_msg.channels[1].name;
	ch2_.name = original_msg.channels[2].name;

	for (i = 0; i < pointCloudNum; i++)
	{
		//if ((pointCloud[i].z >= (double)maxSlot*2/100)&&(pointCloud[i].z < ((double)maxSlot+1)*2/100))
		if (pointCloud[i].z >= ((double)maxSlot+1)*2/100)
		{
			p_.x = pointCloud[i].x;
			p_.y = pointCloud[i].y;
			p_.z = pointCloud[i].z;
			ch0_.values.push_back(original_msg.channels[0].values[i]);
			ch1_.values.push_back(original_msg.channels[1].values[i]);
			ch2_.values.push_back(original_msg.channels[2].values[i]);
			filtered_msg->points.push_back(p_);
			j++;
			
			if (j > newPointNum) 
			{
			//	cout << "j>newPointNum" << endl;
			//	break;
			}
		}
	}
	filtered_msg->channels.push_back(ch0_);
	filtered_msg->channels.push_back(ch1_);
	filtered_msg->channels.push_back(ch2_);

	vector<long unsigned int> clusterId;
	int k = 6000;
	find_cluster(filtered_msg->points, k, clusterId);
	for (i = 0; i < k; i++)
	{
		filtered_msg->channels[0].values[clusterId[i]] = 1;
	//	filtered_msg->channels[1].values[clusterId[i]] = 0.5;
	//	filtered_msg->channels[2].values[clusterId[i]] = 0.5;
	}
}

