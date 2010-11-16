#include "recyclerbot/table_detector.h"
#define RESOLUTION 0.01

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
  
  // count along z axis
	int* countAlongZ;
	int slotNum = int((maxZ - minZ) / RESOLUTION) + 1;
	countAlongZ = new int[slotNum];
	for (i = 0; i < slotNum; i++) countAlongZ[i] = 0;
	  
	for (i = 0; i < pointNum; i++) 
  {
		countAlongZ[int((pointCloud[i].z - minZ) / RESOLUTION)]++;
	}
	
	// find the slot with maximum points
	int maxSlot = 0;
	for (i = 0; i < slotNum; i++) 
	{
		if (countAlongZ[i] > countAlongZ[maxSlot])
		{
			maxSlot = i;
		}
	}
	//cout<<"maxSlot: "<<maxSlot<<endl;
	
	// find the scope of table
	double tableMinX = 100;
	double tableMinY = 100;
	double tableMaxX = -100;
	double tableMaxY = -100;
	
	
	for (i = 0; i < pointNum; i++)
	{
		if (int((pointCloud[i].z - minZ) / RESOLUTION) == maxSlot)
		{
			if (pointCloud[i].x > tableMaxX) tableMaxX = pointCloud[i].x;
			if (pointCloud[i].x < tableMinX) tableMinX = pointCloud[i].x;
			if (pointCloud[i].y > tableMaxY) tableMaxY = pointCloud[i].y;
			if (pointCloud[i].y < tableMinY) tableMinY = pointCloud[i].y;
		}
	}
	// adjust the boundary by leaving a margin
	tableMaxX -= 0.03;
	tableMinX += 0.03;
	tableMaxY -= 0.03;
	tableMinY += 0.03;
	
	sensor_msgs::ChannelFloat32 ch0_, ch1_, ch2_;
	ch0_.name = nt_msg.channels[0].name;
	ch1_.name = nt_msg.channels[1].name;
	ch2_.name = nt_msg.channels[2].name;

	double tabletop = ((double)maxSlot + 2) * RESOLUTION + minZ;
	
	for (i = 0; i < pointNum; i++)
	{
		if ((pointCloud[i].z >= tabletop) && (pointCloud[i].x > tableMinX) && (pointCloud[i].x < tableMaxX) && (pointCloud[i].y < tableMaxY) &&(pointCloud[i].y > tableMinY))
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
	
	//filtered_msgs.push_back(pointsAboveTable);
	
	////////////////////////////////////////////////////////////
	// -------------------SECOND STEP-------------------------//
	// find cylinders after table removed                     //
	////////////////////////////////////////////////////////////

	vector<long unsigned int> clusterId;
	vector<long unsigned int> preclusterId;
	
	
	
	int k = 5;
	
	/*
	find_cluster(pointsAboveTable.points, k, clusterId);


	std_msgs::ColorRGBA tcolor_ = colorArray[0];
	for (j = 0; j < (int)clusterId.size(); j++)
	{
		pointsAboveTable.channels[0].values[clusterId[j]]=tcolor_.r;
		pointsAboveTable.channels[1].values[clusterId[j]]=tcolor_.g;
		pointsAboveTable.channels[2].values[clusterId[j]]=tcolor_.b;
	}


	filtered_msgs.push_back(pointsAboveTable);
	return;
*/



	vector<geometry_msgs::Point32> tempCloud;
	sensor_msgs::PointCloud cylCloud;
	vector<cylinder> cylinders;
	cylinder cylinder_;
	sensor_msgs::PointCloud displayCloud;
	
	for (int cylId = 0; cylId < MAX_CYL_NUM; cylId++)
	{
		find_cluster(pointsAboveTable.points, k, preclusterId);
		
		
		
		if (cylId == 1) 
		{
			std_msgs::ColorRGBA tcolor_ = colorArray[0];
			ch0_.name = 'r';
			ch1_.name = 'g';
			ch2_.name = 'b';
			
			ch0_.values.clear();
			ch1_.values.clear();
			ch2_.values.clear();
	
			//for (j = 0; j < (int)preclusterId.size(); j++)
			for (j = 0; j < (int)pointsAboveTable.points.size(); j++)
			{
				//displayCloud.points.push_back(pointsAboveTable.points[preclusterId[j]]);
				displayCloud.points.push_back(pointsAboveTable.points[j]);
				ch0_.values.push_back(tcolor_.r);
				ch1_.values.push_back(tcolor_.g);
				ch2_.values.push_back(tcolor_.b);
			}
			displayCloud.channels.push_back(ch0_);
			displayCloud.channels.push_back(ch1_);
			displayCloud.channels.push_back(ch2_);
			filtered_msgs.push_back(displayCloud);
		}
		
		
		
		// if cluster is too small, ignore and stop searching
		if (preclusterId.size() < 1000) 
		{
			cout<<"case 1 no cluster: "<<preclusterId.size()<<endl;
			cylNum = cylId;
			break;
		}
		
		// find a bottle
		find_cylinder(pointsAboveTable.points, preclusterId, cylinder_, clusterId);
		
		// if cylinder is too small, ignore and stop searching
		if (clusterId.size() < CYL_POINT_THRESH) 
		{
			cout<<"case 2 no cylinder: "<<clusterId.size()<<endl;
			cylNum = cylId;
			break;
		}
		
		cylinders.push_back(cylinder_);
		
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
		
		cout<<"cylinder "<<cylId<<": "<<clusterId.size()<<endl;
		
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
			if ((p.z > cylinder_.center.z + cylinder_.height / 2) && (distance(cylinder_.center, p) < cylinder_.radius*0.6))
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
		else if (height_ < 0.20) cylinders[i].classId = 1;
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
	
//	filtered_msgs.push_back(cylCloud);
}


void TableDetector::find_cylinder(
vector<geometry_msgs::Point32>& pointCloud,
vector<long unsigned int>& preclusterId,
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
	int clusterSize = preclusterId.size();
	geometry_msgs::Point32 center;
	double r;
	
	for (i = 0; i < iter; i++)
	{
		// pick three random points
		p1 = int(__drand48__() * (clusterSize - 1));
		p2 = int(__drand48__() * (clusterSize - 1));
		p3 = int(__drand48__() * (clusterSize - 1));
		if ((p1 == p2) || (p1 == p3) || (p2 == p3)) {i--; continue;}
		
		// find the circle
		find_circle(pointCloud[preclusterId[p1]], pointCloud[preclusterId[p2]], pointCloud[preclusterId[p3]], center_t, r_t);
		
		// evaluate this circle
		if ((r_t > 0.04)||(r_t < 0.02)) {i--; continue;}
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
		if (consensus > int(preclusterId.size()*0.6)) {cout<<"**consensus: "<<consensus<<endl; break;}
	}
	
	// fit the points to a cylinder, multiple times
	for (int fitTime = 0; fitTime < 2; fitTime++)
	{
		vector<geometry_msgs::Point32> cylPoints;
	
		for (j = 0; j < size; j++)
		{
			error = distance(pointCloud[j], center) - r;
			double amp = 1.5 + (double)fitTime * 0.5;
			if ((error < margin*amp) && (error > -margin*amp))
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

void TableDetector::find_cluster(
vector<geometry_msgs::Point32>& pointCloud, 
int k, 
vector<long unsigned int>& clusterId)
{
	int i,j;
	vector<Point> points;
	Point p_;
	vector<double> distance;
	int size = pointCloud.size();
	int startPoint = 0;
	int* pointStatus = new int[size]; // 0 - unreached, 1 - open, 2 - explored
	bool* pointAssigned = new bool[size];
	
	double disThresh = 0.0004;
	vector<long unsigned int> neighborPoints;
	int clusterMinSize = 2000;
	vector<unsigned long> openList;
	int curId = 0;
	int iter = 0;
	int maxIter = 0.01*size;
	vector<long unsigned int> tempCluster;
	int assignedNum = 0;
	
	if (size < clusterMinSize)
	{
		clusterId.clear();
		return;
	}
	
	// initialize the points
	for (i = 0; i < size; i++)
	{
		pointStatus[i] = 0;
		pointAssigned[i] = false;
		p_[0] = pointCloud[i].x;
		p_[1] = pointCloud[i].y;
		p_[2] = pointCloud[i].z;
		points.push_back(p_);
	}
	
	// initialize the NN algorithm
	sfcnn<Point, 3, double> NN(&points[0], size);

	clusterId.clear();
	do
	{
		iter++;
		for (j = 0; j < size; j++) pointStatus[j] = 0;
		openList.clear();
		tempCluster.clear();
		
		for (i = 0; i < size; i++) 
		{
			if (pointAssigned[i] == false) break;
		}
		
		// set starting point
	/*	do{
			startPoint = int(__drand48__() * (size - 1));
		} while (pointAssigned[startPoint] == true);*/
		startPoint = i;
		openList.push_back(startPoint);
		pointAssigned[startPoint] = true;
		assignedNum++;
		
		do
		{
			// add explored point into cluster
			curId = openList.back();
			openList.pop_back();
			pointStatus[curId] = 2;
			tempCluster.push_back(curId);
			
			// find neighbor points
			NN.ksearch(points[curId], k, neighborPoints, distance);

		//	if (distance[0] > disThresh) continue;
			
			// add new points into open list
			for (j = 0; j < k; j++)
			{
				curId = neighborPoints[j];
				if ((pointStatus[curId] == 0)&&(distance[j] < disThresh))
				{
					openList.push_back(curId);
					pointStatus[curId] = 1;
					if (pointAssigned[curId] == false) assignedNum++;
					pointAssigned[curId] = true;
				}
			}
			
		} while (!openList.empty());
		
		if (tempCluster.size() > max(clusterMinSize, (int)clusterId.size())) clusterId = tempCluster;
	} while (assignedNum < size);//(clusterId.size() < clusterMinSize));// && (iter <= maxIter));
	
	if (iter >= maxIter)
	{
		cout<<"no more cluster, iter: "<<iter<<" size: "<<clusterId.size()<<endl;
		clusterId.clear();
	}
	
	cout<<"cluster size: "<<clusterId.size()<<endl;
	delete pointStatus;
}



