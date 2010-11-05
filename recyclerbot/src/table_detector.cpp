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
sensor_msgs::PointCloud* filtered_msg,
visualization_msgs::Marker* marker_msg
)
{
	vector<geometry_msgs::Point32>& pointCloud = original_msg.points;
	sensor_msgs::PointCloud pointsAboveTable;
  int i = 0;
	int j = 0;  
  double maxZ = 0;
  double minZ = 100;
  int pointCloudNum = pointCloud.size();
  int* assigned = NULL;
  
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
		if (pointCloud[i].z >= ((double)maxSlot + 1) * 2 / 100)
		{
			p_.x = pointCloud[i].x;
			p_.y = pointCloud[i].y;
			p_.z = pointCloud[i].z;
			ch0_.values.push_back(original_msg.channels[0].values[i]);
			ch1_.values.push_back(original_msg.channels[1].values[i]);
			ch2_.values.push_back(original_msg.channels[2].values[i]);
			pointsAboveTable.points.push_back(p_);
			j++;
			
		}
	}
	pointsAboveTable.channels.push_back(ch0_);
	pointsAboveTable.channels.push_back(ch1_);
	pointsAboveTable.channels.push_back(ch2_);

	// now start to find cylinders!
	vector<long unsigned int> clusterId;
	geometry_msgs::Point32 center;
	double radius = 0;
	
//	int k = 6000;
//	find_cluster(filtered_msg->points, k, clusterId);

	ch0_.name = 'r';
	ch1_.name = 'g';
	ch2_.name = 'b';
	ch0_.values.clear();
	ch1_.values.clear();
	ch2_.values.clear();

	vector<geometry_msgs::Point32> tempCloud;
	for (int cylId = 0; cylId < 3; cylId++)
	{
		// find first bottle
		find_cylinder(pointsAboveTable.points, center, radius, clusterId);
		// set cylinder marker
		marker_msg[cylId].pose.position.x = center.x;
		marker_msg[cylId].pose.position.y = center.y;
		marker_msg[cylId].pose.position.z = center.z;
		marker_msg[cylId].pose.orientation.x = 0.0;
		marker_msg[cylId].pose.orientation.y = 0.0;
		marker_msg[cylId].pose.orientation.z = 0.0;
		marker_msg[cylId].pose.orientation.w = 1.0;
		marker_msg[cylId].scale.x = radius * 2;
		marker_msg[cylId].scale.y = radius * 2;
		marker_msg[cylId].scale.z = 0.1;
		
		// set channels
		int filterPointNum = pointsAboveTable.points.size();
		if (assigned != NULL)	delete assigned;
		assigned = new int[filterPointNum];
		for (i = 0; i < filterPointNum; i++) assigned[i] = 0;
	
		for (i = 0; i < clusterId.size(); i++)
		{
			filtered_msg->points.push_back(pointsAboveTable.points[clusterId[i]]);
			ch0_.values.push_back(cylId == 0 ? 1:0);
			ch1_.values.push_back(cylId == 1 ? 1:0);
			ch2_.values.push_back(cylId == 2 ? 1:0);
			assigned[clusterId[i]] = 1;
		}
		
		// update point cloud
		tempCloud.clear();
		for (i = 0; i < filterPointNum; i++)
		{
			if (assigned[i] == 0) tempCloud.push_back(pointsAboveTable.points[i]);
		}
	
		pointsAboveTable.points.clear();
		pointsAboveTable.points = tempCloud;
	}
	
	filtered_msg->channels.push_back(ch0_);
	filtered_msg->channels.push_back(ch1_);
	filtered_msg->channels.push_back(ch2_);	
	
	cout<<"remaining points: "<<tempCloud.size()<<endl;
	
}

void TableDetector::find_cylinder(vector<geometry_msgs::Point32>& pointCloud,
								   geometry_msgs::Point32& center,
								   double& r,
									 vector<long unsigned int>& clusterId)
{
	// RANSAC on 3 points to fit circle model
	int iter = 2000;
	int i = 0;
	int j = 0;
	int p1, p2, p3;
	int consensus = 0;
	int consensus_t = 0; // temp variable
	geometry_msgs::Point32 center_t; // temp variable
	double r_t = 0; // temp variable
	double error = 0;
	double margin = 0.005;
	int size = pointCloud.size();
	
	for (i = 0; i < iter; i++)
	{
		// pick three random points
		p1 = int(__drand48__() * (size - 1));
		p2 = int(__drand48__() * (size - 1));
		p3 = int(__drand48__() * (size - 1));
		if ((p1 == p2) || (p1 == p3) || (p2 == p3)) {i--;continue;}
		// find the circle
		find_circle(pointCloud[p1], pointCloud[p2], pointCloud[p3], center_t, r_t);
		// evaluate this circle
		if (r_t > 0.07) {i--;continue;}
		consensus_t = 0;
		for (j = 0; j < size; j++)
		{
			error = distance(pointCloud[j], center_t) - r_t;
			if ((error < margin) && (error > -margin)) consensus_t++;
		}
		
		
		
		
		// update center and radius
		if (consensus_t > consensus)
		{
			center.x = center_t.x;
			center.y = center_t.y;			
			center.z = (pointCloud[p1].z + pointCloud[p2].z + pointCloud[p3].z) / 3;
			r = r_t;
			consensus = consensus_t;
		}
	}
			
	clusterId.clear();
	for (j = 0; j < size; j++)
	{
		error = distance(pointCloud[j], center) - r;
		if ((error < margin*2) && (error > -margin*2))
		{
			clusterId.push_back(j);
		}
	}
}



