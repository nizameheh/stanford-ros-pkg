#include "recyclerbot/table_detector.h"
#define RESOLUTION 0.02

using namespace std;

TableDetector::TableDetector()
{
	// initialize color array
	for (int i = 0; i < COLORNUM; i++)
	{
		colorArray[i].r = color_array[i][0];
		colorArray[i].g = color_array[i][1];
		colorArray[i].b = color_array[i][2];
		colorArray[i].a = color_array[i][3];
	}
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
sensor_msgs::PointCloud& nt_msg,
sensor_msgs::PointCloud& ws_msg,
int& cylNum,
vector<sensor_msgs::PointCloud>& filtered_msgs,
vector<visualization_msgs::Marker>& marker_msgs
)
{
	vector<geometry_msgs::Point32>& pointCloud = nt_msg.points;
	sensor_msgs::PointCloud pointsAboveTable;
  int i = 0;
	int j = 0;  
  double maxZ = 0;
  double minZ = 100;
  int pointNum = pointCloud.size();
  int* assigned = NULL;
  
	////////////////////////////////////////////////////////////
	// -------------------FIRST STEP--------------------------
	// remove table
	////////////////////////////////////////////////////////////
	
  // find max and min z value
  for (i = 0; i < pointNum; i++) 
  {
  	if (pointCloud[i].z > maxZ) maxZ = pointCloud[i].z;
  	if (pointCloud[i].z < minZ) minZ = pointCloud[i].z;
  }
  //cout << maxZ << "; " << minZ << endl;
  // count along z axis
	int* countAlongZ;
	int slotNum = int((maxZ - minZ) / RESOLUTION) + 1;
	countAlongZ = new int[slotNum];
	for (i = 0; i < slotNum; i++) countAlongZ[i] = 0;
	  
	for (i = 0; i < pointNum; i++) 
  {
		countAlongZ[int((pointCloud[i].z - minZ) / RESOLUTION)]++;
	}
	//for (i = 0; i < slotNum; i++) cout<<i<<": " << countAlongZ[i] << endl;
	int maxSlot = 0;
	for (i = 0; i < slotNum; i++) 
	{
		if (countAlongZ[i] > countAlongZ[maxSlot])
		{
			maxSlot = i;
		}
	}
	//cout<<"maxSlot: "<<maxSlot<<endl;
	
	sensor_msgs::ChannelFloat32 ch0_, ch1_, ch2_;
	ch0_.name = nt_msg.channels[0].name;
	ch1_.name = nt_msg.channels[1].name;
	ch2_.name = nt_msg.channels[2].name;

	double tabletop = ((double)maxSlot + 1) * RESOLUTION + minZ;
	for (i = 0; i < pointNum; i++)
	{
		//if ((pointCloud[i].z >= (double)maxSlot*2/100)&&(pointCloud[i].z < ((double)maxSlot+1)*2/100))
		if (pointCloud[i].z >= tabletop)
		{
			ch0_.values.push_back(nt_msg.channels[0].values[i]);
			ch1_.values.push_back(nt_msg.channels[1].values[i]);
			ch2_.values.push_back(nt_msg.channels[2].values[i]);
			pointsAboveTable.points.push_back(pointCloud[i]);			
		}
	}
	pointsAboveTable.channels.push_back(ch0_);
	pointsAboveTable.channels.push_back(ch1_);
	pointsAboveTable.channels.push_back(ch2_);
	
	
	////////////////////////////////////////////////////////////
	// -------------------SECOND STEP--------------------------
	// find cylinders after table removed
	////////////////////////////////////////////////////////////

	vector<long unsigned int> clusterId;
	
//	int k = 6000;
//	find_cluster(filtered_msg->points, k, clusterId);

	vector<geometry_msgs::Point32> tempCloud;
	sensor_msgs::PointCloud cylCloud;
	vector<cylinder> cylinders;
	cylinder cylinder_;
	
	for (int cylId = 0; cylId < MAX_CYL_NUM; cylId++)
	{
		// find first bottle
		find_cylinder(pointsAboveTable.points, cylinder_, clusterId);
		
		cylinders.push_back(cylinder_);
		
		// if cylinder is too small, ignore and stop searching
		if (clusterId.size() < CYL_POINT_THRESH) 
		{
			cylNum = cylId;
			break;
		}
		
		// construct clouds
		int filterPointNum = pointsAboveTable.points.size();
		
		if (assigned != NULL)	delete assigned;
		assigned = new int[filterPointNum];
		for (i = 0; i < filterPointNum; i++) assigned[i] = 0;
	
		for (i = 0; i < cylinder_.pointNum; i++)
		{
			cylCloud.points.push_back(pointsAboveTable.points[clusterId[i]]);
			assigned[clusterId[i]] = 1;
		}
		
		// update point cloud
		tempCloud.clear();
		for (i = 0; i < filterPointNum; i++)
		{
			if (assigned[i] == 0) tempCloud.push_back(pointsAboveTable.points[i]);
		}
		
		cout<<cylId<<" cylinder: "<<clusterId.size()<<endl;
		
		pointsAboveTable.points.clear();
		pointsAboveTable.points = tempCloud;
	}
	
	
	cout<<"remaining points: "<<tempCloud.size()<<endl;
	cout<<"-----------------------------------------"<<endl;
	
	
	////////////////////////////////////////////////////////////
	// -------------------THIRD STEP--------------------------
	// find bottle cap
	////////////////////////////////////////////////////////////
	
	vector<geometry_msgs::Point32>& wsPointCloud = ws_msg.points;
	int wsSize = wsPointCloud.size();
	geometry_msgs::Point32 p;
	
	vector<sensor_msgs::PointCloud> caps;
	sensor_msgs::PointCloud cap;
	for (i = 0; i < cylNum; i++) caps.push_back(cap);
	
	for (i = 0; i < wsSize; i++)
	{
		p = wsPointCloud[i];
		for (j = 0; j < cylNum; j++)
		{
			cylinder_ = cylinders[j];
			if ((p.z > cylinder_.center.z + cylinder_.height / 2) && (distance(cylinder_.center, p) < cylinder_.radius))
			{
				caps[j].points.push_back(p);	
			}
		}
	}
	
	////////////////////////////////////////////////////////////
	// -------------------FORTH STEP--------------------------
	// distinguish between cans            (about 13cm),
	//                     glass bottles   (about 20cm)
	//                     plastic bottles (about 22cm)
	////////////////////////////////////////////////////////////
	
	for (i = 0; i < cylNum; i++)
	{
		// find the height of upper part
		cylinder_ = cylinders[i];
		double topZ = cylinder_.center.z + cylinder_.height / 2;
		for (j = 0; j < int(caps[i].points.size()); j++)
		{
			if (caps[i].points[j].z > topZ) topZ = caps[i].points[j].z;
		}
		
		double height_ = topZ - tabletop;
		
		if (height_ < 0.15) cylinders[i].classId = 0;
		else if (height_ < 0.21) cylinders[i].classId = 1;
		else cylinders[i].classId = 2;
	}
	
	////////////////////////////////////////////////////////////
	// -------------------FIFTH STEP--------------------------
	// draw markers and color the point cloud
	////////////////////////////////////////////////////////////
	
	ch0_.name = 'r';
	ch1_.name = 'g';
	ch2_.name = 'b';
	ch0_.values.clear();
	ch1_.values.clear();
	ch2_.values.clear();
	
	cylCloud.channels.push_back(ch0_);
	cylCloud.channels.push_back(ch1_);
	cylCloud.channels.push_back(ch2_);
	
	std_msgs::ColorRGBA color_;
	
	for (i = 0; i < cylNum; i++)
	{
		// construct and configure a new marker
		visualization_msgs::Marker marker_;
		
		cylinder_ = cylinders[i];
		color_ = colorArray[cylinders[i].classId];
			
		marker_.header.frame_id = "/base_footprint";
		marker_.header.stamp = ros::Time::now();
		marker_.ns = "bottle";
		marker_.id = i;
		marker_.type = visualization_msgs::Marker::CYLINDER;
		marker_.action = visualization_msgs::Marker::ADD;
		marker_.color = color_;
		marker_.color.a = 0.5;
		marker_.lifetime = ros::Duration();
		marker_.pose.position.x = cylinder_.center.x;
		marker_.pose.position.y = cylinder_.center.y;
		marker_.pose.position.z = cylinder_.center.z;
		marker_.pose.orientation.x = 0.0;
		marker_.pose.orientation.y = 0.0;
		marker_.pose.orientation.z = 0.0;
		marker_.pose.orientation.w = 1.0;
		marker_.scale.x = cylinder_.radius * 2;
		marker_.scale.y = cylinder_.radius * 2;
		marker_.scale.z = cylinder_.height;
		
		marker_msgs.push_back(marker_);
		
		for (j = 0; j < cylinder_.pointNum; j++)
		{
			cylCloud.channels[0].values.push_back(color_.r);
			cylCloud.channels[1].values.push_back(color_.g);
			cylCloud.channels[2].values.push_back(color_.b);
		}
		
	}
	for (i = 0; i < cylNum; i++)
	{
		color_ = colorArray[cylinders[i].classId];
		for (j = 0; j < int(caps[i].points.size()); j++)
		{
			cylCloud.points.push_back(caps[i].points[j]);
			cylCloud.channels[0].values.push_back(color_.r);
			cylCloud.channels[1].values.push_back(color_.g);
			cylCloud.channels[2].values.push_back(color_.b);
		}
	}
	
	filtered_msgs.push_back(cylCloud);
}


void TableDetector::find_cylinder(
vector<geometry_msgs::Point32>& pointCloud,
cylinder& cylinder_,
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
	geometry_msgs::Point32 center;
	double r;
	
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
	
	// fit the points to a cylinder, twice
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
	cylinder_.height = topz - bottomz;
	cylinder_.center = center;
	cylinder_.radius = r;
	cylinder_.pointNum = clusterId.size();
}

void TableDetector::fit_circle(
vector<geometry_msgs::Point32>& pointCloud,
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
	geometry_msgs::Point32 p;
	
	for (i = 0; i < size; i++)
	{
		p = pointCloud[i];
		X1 = X1 + p.x;
		Y1 = Y1 + p.y;
		X2 = X2 + p.x*p.x;
		Y2 = Y2 + p.y*p.y;
		X3 = X3 + p.x*p.x*p.x;
		Y3 = Y3 + p.y*p.y*p.y;
		X1Y1 = X1Y1 + p.x*p.y;
		X1Y2 = X1Y2 + p.x*p.y*p.y;
		X2Y1 = X2Y1 + p.x*p.x*p.y;
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



