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
vector<visualization_msgs::Marker>& marker_msg
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
	double height = 0;
	
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
		find_cylinder(pointsAboveTable.points, center, radius, height, clusterId);
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
		marker_msg[cylId].scale.z = height;
		
		// set channels
		int filterPointNum = pointsAboveTable.points.size();
		if (assigned != NULL)	delete assigned;
		assigned = new int[filterPointNum];
		for (i = 0; i < filterPointNum; i++) assigned[i] = 0;
	
		for (i = 0; i < clusterId.size(); i++)
		{
			filtered_msg->points.push_back(pointsAboveTable.points[clusterId[i]]);
			ch0_.values.push_back(marker_msg[cylId].color.r);
			ch1_.values.push_back(marker_msg[cylId].color.g);
			ch2_.values.push_back(marker_msg[cylId].color.b);
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
								   double& h,
									 vector<long unsigned int>& clusterId)
{
	// RANSAC on 3 points to fit circle model
	int iter = 1000;
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
		if ((p1 == p2) || (p1 == p3) || (p2 == p3)) {i--; continue;}
		
		// find the circle
		find_circle(pointCloud[p1], pointCloud[p2], pointCloud[p3], center_t, r_t);
		
		// evaluate this circle
		if ((r_t > 0.07)||(r_t < 0.03)) {i--; continue;}
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
			r = r_t;
			consensus = consensus_t;
		}
	}
	
	// fit the points to a cylinder
	for (int fitTime = 0; fitTime < 2; fitTime++)
	{
		vector<geometry_msgs::Point32> cylPoints;
	
		for (j = 0; j < size; j++)
		{
			error = distance(pointCloud[j], center) - r;
			if ((error < margin*2) && (error > -margin*2))
			{
				cylPoints.push_back(pointCloud[j]);
			}
		}
		fit_circle(cylPoints, center, r);
	}
	// update the cylinder cluster index and find center z, height
	double topz = 0;
	double bottomz = 10;
	
	clusterId.clear();
	for (j = 0; j < size; j++)
	{
		error = distance(pointCloud[j], center) - r;
		if ((error < margin*2) && (error > -margin*2))
		{
			clusterId.push_back(j);
			if (topz < pointCloud[j].z) topz = pointCloud[j].z;
			if (bottomz > pointCloud[j].z) bottomz = pointCloud[j].z;
		}
	}
	center.z = (topz + bottomz) / 2;
	h = topz - bottomz;
}

void TableDetector::fit_circle(vector<geometry_msgs::Point32>& pointCloud,
															 geometry_msgs::Point32& center,
															 double& r)
{
	int size = pointCloud.size();
	int i = 0;
	
	double X1 = 0;
	double Y1 = 0;
	double X2 = 0;
	double Y2 = 0;
	double X3 = 0;
	double Y3 = 0;
	double X1Y1=0;
	double X1Y2 = 0;
	double X2Y1 = 0;
	
	for (i = 0; i < size; i++)
	{
		X1 = X1 + pointCloud[i].x;
		Y1 = Y1 + pointCloud[i].y;
		X2 = X2 + pointCloud[i].x*pointCloud[i].x;
		Y2 = Y2 + pointCloud[i].y*pointCloud[i].y;
		X3 = X3 + pointCloud[i].x*pointCloud[i].x*pointCloud[i].x;
		Y3 = Y3 + pointCloud[i].y*pointCloud[i].y*pointCloud[i].y;
		X1Y1 = X1Y1 + pointCloud[i].x*pointCloud[i].y;
		X1Y2 = X1Y2 + pointCloud[i].x*pointCloud[i].y*pointCloud[i].y;
		X2Y1 = X2Y1 + pointCloud[i].x*pointCloud[i].x*pointCloud[i].y;
	}
	
	double C,D,E,G,H,N;
	double a,b,c;
	N = size;
	C = N*X2 - X1*X1;
	D = N*X1Y1 - X1*Y1;
	E = N*X3 + N*X1Y2 - (X2+Y2)*X1;
	G = N*Y2 - Y1*Y1;
	H = N*X2Y1 + N*Y3 - (X2+Y2)*Y1;
	a = (H*D-E*G)/(C*G-D*D);
	b = (H*C-E*D)/(D*D-G*C);

	c = -(a*X1 + b*Y1 + X2 + Y2)/N;

	double A,B,R;
	A = a/(-2);
	B = b/(-2);
	R = sqrt(a*a+b*b-4*c)/2;
	
	center.x = A;
	center.y = B;
	r = R;
}



